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

class RecoveryExecutable(Node):
    def __init__(self):
        super().__init__('recovery_executable')
        self.publisher_Twist = self.create_publisher(Twist, 'recovery_cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.velocity_publishing_timer = self.create_timer(timer_period, self.velocity_publishing)
        self.back_up_velocity_timer = self.create_timer(timer_period, self.set_velocity_from_error)
        self.sweep_timer = self.create_timer(timer_period, self.sweep_left_and_right)
        self.arduino_timer = self.create_timer(0.05, self.updateArduinoReading)


        self.subscription = self.create_subscription(String, 'state', self.listener_callback, QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),)

        self.ultraSoundReadingFloat = 2000.0
        self.ultraSoundReadingHistory = []  # Store last 5 readings for median filter
        self.targetLinearVelocity = 0.0 #m/s???
        self.targetAngularVelocity = 0.0 #rad/s???
        self.targetPosition = 0.30 #meters
        self.distanceTraveled = 0.0
        self.proportional = 0.333
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

    #This gets in the arduino reading and updates the self.ultraSoundReadingFloat, which is where we use the ultrasound readigns in the rest of the code
    def updateArduinoReading(self):
        if self.arduino is None:
            return

        try:
            if self.arduino.in_waiting > 0:
                packet = self.arduino.readline()
                ultraSoundReadingString = packet.decode('utf-8', errors='ignore').rstrip('\n')
                try:
                    raw_reading = float(ultraSoundReadingString)
                    self.ultraSoundReadingHistory.append(raw_reading)
                    if len(self.ultraSoundReadingHistory) > 5:
                        self.ultraSoundReadingHistory.pop(0)
                    self.ultraSoundReadingFloat = median(self.ultraSoundReadingHistory)
                    if self.state == "BeginRecovery":
                        self.state = "temp"
                        self.begin_recovery()
                except:
                    self.get_logger().info("not reading ultrasound values")

        except OSError as e:
            self.get_logger().error(f"Arduino disconnected: {e}")
            self.arduino = None
            if self.state != "NoPublishing":
                self.get_logger().info("Calling end_recovery due to Arduino disconnect")
                self.end_recovery()
                
    def listener_callback(self, msg): #Note from Hannah & Ash: How does msg variable work, and does it conflict with msg var in twist?
        if msg.data == 'recovery':
            self.state = "BeginRecovery"

    #This funciton is called periodically. It actually publishes the velocity messages and has the logic of whether to call the function that just drives teh robot backwards or call the sweep function
    def velocity_publishing(self):
        #self.get_logger().info(f'calling timer callback')
        self.get_logger().info(f'ultrasoundreading: {self.ultraSoundReadingFloat}')

    #this is where the velocity messages are actually published
        msg = Twist()
        msg.linear.x = -self.targetLinearVelocity
        msg.angular.z = self.targetAngularVelocity
        #self.get_logger().info(str(msg))
        if self.state != "NoPublishing":
            self.publisher_Twist.publish(msg)
        #self.get_logger().info(f'ultrasoundreading: {self.ultraSoundReadingFloat}')

    #this function sets the velocity to have the robot back up. The velocity starts large, then decreases as the robot gets closer to its target
    #currently, it is set to backup 0.30 meters
    def set_velocity_from_error(self):

        if self.state == "beginBackUp":
            self.get_logger().info("backing up")
            self.distanceTraveled = self.distanceTraveled + (0.5 * self.targetLinearVelocity)
            error = self.targetPosition - self.distanceTraveled
            self.targetLinearVelocity = self.proportional * error
            if error < 0.01:
                self.end_recovery()
            self.get_logger().info(f'distancetraveled: {self.distanceTraveled}')
            #self.get_logger().info(f'error: {error}')
            self.get_logger().info(f'targetLinearVelocity: {self.targetLinearVelocity}')

#this function sets the robot's angular velocity to sweep left and right if it senses something directly behind it
#the logic in the timer callback function should stop it from running once it doesn't sense something behind it
    def sweep_left_and_right(self):
        #self.get_logger().info(f'sweeping function called')
        if self.state != "beginSweep" and self.state != "flipDirectionSweep" and self.state != "backUpAfterSweep":
            return
        if self.ultraSoundReadingFloat >= 40:
            self.state = "beginBackUp"
        elif self.state == "beginSweep":
            self.get_logger().info(f'sweeping counterclockwise')
            if self.radiansTravelled < 0.349066:#sweeps 20 degrees one direction
            #sweeps left pi/2 radians or 90 degrees one way at a speed of pi/9 radians per second
                self.targetAngularVelocity = math.pi / 9
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * 0.5)
            else: 
                self.state = "flipDirectionSweep"
        elif self.state == "flipDirectionSweep":
            if self.radiansTravelled > -0.349066:
                self.get_logger().info(f'sweeping clockwise')

                self.targetAngularVelocity = -1 * math.pi / 9
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * 0.5)
            else:
                self.state = "backUpAfterSweep"
                self.targetAngularVelocity = 0.0
                self.get_logger().info(f'sweeping ended')

        if self.state ==  "backUpAfterSweep":
            if(self.ultraSoundReadingFloat > 10):
                self.get_logger().info(f'backing up within 10 centimeters of obstacle')
                self.targetLinearVelocity = 0.02 #sets velocity
            else:
                self.end_recovery()

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
        # boolmsg = Bool()
        # boolmsg.data = False
        self.get_logger().info(f'recovery ended')
        response = self.send_request(False)
        # self.publisher_Boolean.publish(boolmsg)

    def begin_recovery(self):
        if self.ultraSoundReadingFloat >= 40.0:
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
