import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy.executors import ExternalShutdownException
import math
from std_srvs.srv import SetBool
import sys
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from statistics import median

recovery_executable = None
ANGULAR_VELOCITY = 0.3 #radians per second
MAX_ANGULAR_SWEEP_DISTANCE = 0.349066 #radians
THRESHOLD_DISTANCE = 60 #centimeters, distance needed to start backing up
MAX_BACKUP_DISTANCE = 0.50 #meters
BACKUP_ALLOWANCE = 15 #centimeters, if the ultrasound reader reads less than this, the robot stops backing up and ends recovery
PROPORTIONAL = 0.333 #the p term for the fake p loop 

class RecoveryExecutable(Node):
    def __init__(self):
        super().__init__('recovery_executable')
        self.publisher_Twist = self.create_publisher(Twist, 'recovery_cmd_vel', 10)
        self.timer_period = 0.2  # seconds
        self.velocity_publishing_timer = self.create_timer(self.timer_period, self.velocity_publishing)
        self.back_up_velocity_timer = self.create_timer(self.timer_period, self.set_velocity_from_error)
        self.sweep_timer = self.create_timer(self.timer_period, self.sweep_left_and_right)
        self.arduino_timer = self.create_timer(0.05, self.updateArduinoReading)

        self.subscription = self.create_subscription(String, 'state', self.listener_callback, QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),)

        self.ultrasoundReading = 2000.0
        self.ultrasoundReadingHistory = []  # Store last 5 readings for median filter
        self.targetLinearVelocity = 0.0 #m/s
        self.targetAngularVelocity = 0.0 #rad/s
        self.distanceTraveled = 0.0
        self.radiansTravelled = 0.0
        self.state = "NoPublishing"
       
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port")
            self.arduino = None

        self.cli = self.create_client(SetBool, 'state/set_recovery')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    #This gets in the arduino reading and updates the self.ultrasoundReading, which is where we use the ultrasound readigns in the rest of the code
    def updateArduinoReading(self):
        if self.arduino is None:
            return
        try:
            latest = None
            while self.arduino.in_waiting > 0:
                packet = self.arduino.readline()
                latest = packet  # keep overwriting — only the last matters
            if latest:
                reading_str = latest.decode('utf-8', errors='ignore').strip()
                try:
                    self.ultrasoundReading = float(reading_str)
                except ValueError:
                    self.get_logger().info("not reading ultrasound values")
        
        except OSError as e:
            self.get_logger().error(f"Arduino disconnected: {e}")
            self.arduino = None
            if self.state != "NoPublishing":
                self.get_logger().info("Calling end_recovery due to Arduino disconnect")
                self.end_recovery()
                    
    def listener_callback(self, msg): 
        if msg.data == 'recovery':
            self.begin_recovery()

    #This funciton is called periodically. It actually publishes the velocity messages and has the logic of whether to call the function that just drives teh robot backwards or call the sweep function
    def velocity_publishing(self):
        self.get_logger().info(f'ultrasoundreading: {self.ultrasoundReading}')

    #this is where the velocity messages are actually published
        msg = Twist()
        msg.linear.x = -self.targetLinearVelocity
        msg.angular.z = self.targetAngularVelocity
        if self.state != "NoPublishing":
            self.publisher_Twist.publish(msg)

    #this function sets the velocity to have the robot back up. The velocity starts large, then decreases as the robot gets closer to its target
    #not a real pid loop, distance traveled is calculated, not read in from sensors
    def set_velocity_from_error(self):
        if self.state == "beginBackUp" and self.ultrasoundReading > BACKUP_ALLOWANCE:
            self.get_logger().info("backing up")
            #calculates how far we have traveled from our speed and the frequency this function is called at
            self.distanceTraveled = self.distanceTraveled + (self.timer_period * self.targetLinearVelocity)
            error = MAX_BACKUP_DISTANCE - self.distanceTraveled
            self.targetLinearVelocity = PROPORTIONAL * error
            #Because this is a calculated error, I don't think this error allowance needs to be tuned very much
            if error < 0.01: 
                self.end_recovery()
            self.get_logger().info(f' calculated distancetraveled: {self.distanceTraveled}')
            #self.get_logger().info(f'error: {error}')
            self.get_logger().info(f'targetLinearVelocity: {self.targetLinearVelocity}')  
        
        #If the ultrasound reader reads less than the backup allowance, it stops backing up and ends recovery
        elif self.state == "beginBackUp" and self.ultrasoundReading <= BACKUP_ALLOWANCE:
            self.get_logger().info(f'saw object within backup allowance, ending backup')
            self.end_recovery()  

#this function sets the robot's angular velocity to sweep left and right if it senses something directly behind it
    def sweep_left_and_right(self):
        if self.state != "beginSweep" and self.state != "flipDirectionSweep":
            return
        
        #if the ultrasound sensor is reading more than the threshold distance, start backing up
        if self.ultrasoundReading >= THRESHOLD_DISTANCE:
            self.targetAngularVelocity = 0.0
            self.state = "beginBackUp"
        elif self.state == "beginSweep":
            self.get_logger().info(f'sweeping counterclockwise')
            #turn one direction until radians travelled exceeds the max distance, then set state to flip direction
            if self.radiansTravelled < MAX_ANGULAR_SWEEP_DISTANCE:
                self.targetAngularVelocity = ANGULAR_VELOCITY
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * self.timer_period)
            else: 
                self.state = "flipDirectionSweep"
        elif self.state == "flipDirectionSweep":
            #turn the other direction until radians traveled, is greater than max distance
            if self.radiansTravelled > -MAX_ANGULAR_SWEEP_DISTANCE:
                self.get_logger().info(f'sweeping clockwise')
                self.targetAngularVelocity = -1 * ANGULAR_VELOCITY
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * self.timer_period)
            #If we have reached the max angular distance allowed in both directions without finding a point with large clearance, start backing up. 
            # backup will continue until the reading is less than the backup allowance
            else:
                self.targetAngularVelocity = 0.0
                self.state = "beginBackUp"
                self.get_logger().info(f'sweeping ended')

    def end_recovery(self):
        self.targetAngularVelocity = 0.0
        self.radiansTravelled = 0.0
        self.targetLinearVelocity = 0.0
        self.distanceTraveled = 0.0
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_Twist.publish(msg)
        self.state = "NoPublishing"
        self.get_logger().info(f'recovery ended')
        response = self.send_request(False)

    def begin_recovery(self):
        if self.ultrasoundReading >= THRESHOLD_DISTANCE:
            self.state = "beginBackUp"
            self.targetAngularVelocity = 0.0

    #this tests if there is an object closer than 40 centimeters, in which case it calls the sweeping method
        else:
            self.targetLinearVelocity = 0.0
            self.state = "beginSweep"

    #This should publish the message back to nav
    def send_request(self, set_recovery):
        self.req.data = set_recovery
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Service response: success={response.success}, message="{response.message}"'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

#this actually runs the code
def main(args=None):
    try:
        rclpy.init(args=args)
        global recovery_executable
        recovery_executable = RecoveryExecutable()

        rclpy.spin(recovery_executable)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()