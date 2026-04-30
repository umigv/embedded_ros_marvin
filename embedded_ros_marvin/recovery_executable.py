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
THRESHOLD_DISTANCE = 60 #centimeters, distance needed to start backing up
BACKUP_ALLOWANCE = 30 #centimeters, if the ultrasound reader reads less than this, the robot stops backing up and ends recovery
BACKUP_VELOCITY = 0.25 #m/s
BACKUP_TIME = 9.0 #seconds
ANGULAR_VELOCITY = 1.0 #radians per second
SWEEP_TIME = 6 #seconds, total time to sweep both directions
SWEEP_PAUSE_TIME = 1.0 #seconds, time to pause in between sweeps and at the end of the sweep before backing up

class RecoveryExecutable(Node):
    def __init__(self):
        super().__init__('recovery_executable')
        self.publisher_Twist = self.create_publisher(Twist, 'recovery_cmd_vel', 10)
        self.timer_period = 0.2  # seconds
        self.velocity_publishing_timer = self.create_timer(self.timer_period, self.velocity_publishing)
        self.backup_timer = self.create_timer(self.timer_period, self.back_up)
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
        self.state = "noPublishing"
        self.totalTime = 0.0
        self.backupEndTime = BACKUP_TIME #seconds, is just a default value, this variable is changed through the node running
        self.leftSweepTime = SWEEP_TIME / 4 #seconds, default value
        self.rightSweepTime = SWEEP_TIME / 2 #seconds, default value
        self.totalSweepTime = SWEEP_TIME #seconds, default value
        self.firstStopTime = SWEEP_PAUSE_TIME #seconds, default value
        self.secondStopTime = SWEEP_PAUSE_TIME #seconds, default value
       
        try:
            self.arduino = serial.Serial('/dev/ultrasonic', 9600, timeout=1)
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
                readingStr = latest.decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"raw ultrasound reading: {readingStr}")
                try:
                    #median value filter
                    rawReading = float(readingStr)
                    self.ultrasoundReadingHistory.append(rawReading)
                    self.get_logger().info(f"ultrasound list {self.ultrasoundReadingHistory}")

                    if len(self.ultrasoundReadingHistory) > 3:
                        self.ultrasoundReadingHistory.pop(0)
                    self.ultrasoundReading = median(self.ultrasoundReadingHistory)     
                except ValueError:
                    self.get_logger().info("not reading ultrasound values")
        
        except OSError as e:
            self.get_logger().error(f"Arduino disconnected: {e}")
            self.arduino = None
            if self.state != "noPublishing":
                self.get_logger().info("Calling end_recovery due to Arduino disconnect")
                self.end_recovery()
                    
    def listener_callback(self, msg): 
        if msg.data == 'recovery':
            self.begin_recovery()

    #This funciton is called periodically. It actually publishes the velocity messages and has the logic of whether to call the function that just drives teh robot backwards or call the sweep function
    def velocity_publishing(self):
        self.get_logger().info(f'ultrasoundreading: {self.ultrasoundReading}')
        #increments the total time variable regardless of whether or not the recovery behavior is running
        self.totalTime += self.timer_period

        #this is where the velocity messages are actually published
        msg = Twist()
        msg.linear.x = -self.targetLinearVelocity
        msg.angular.z = self.targetAngularVelocity
        if self.state != "noPublishing":
            self.publisher_Twist.publish(msg)

    #this function sets the velocity to have the robot back up
    def back_up(self):
        if self.state == "backup":
            if self.ultrasoundReading <= BACKUP_ALLOWANCE:
                self.get_logger().info(f'saw object within backup allowance, ending backup')
                self.end_recovery() 
            elif self.totalTime < self.backupEndTime:
                self.get_logger().info("backing up")
                self.targetLinearVelocity = BACKUP_VELOCITY
            else:
                self.get_logger().info("finished backing up")
                self.end_recovery()
            
#this function sets the robot's angular velocity to sweep left and right if it senses something directly behind it
    def sweep_left_and_right(self):
        if self.state == "startSweep":
            #stops sweeping if the ultrasound reading is big enough or if we have exceeded the total time allowed for sweeping
            if self.ultrasoundReading > THRESHOLD_DISTANCE or self.totalTime >= self.totalSweepTime:
                self.targetAngularVelocity = 0.0
                self.get_logger().info(f'ending sweep')
                #updates time to stop backing up
                self.backupEndTime = self.totalTime + BACKUP_TIME
                self.state = "backup"
                
            elif self.totalTime < self.leftSweepTime:                
                self.get_logger().info(f'sweeping counterclockwise')
                self.targetAngularVelocity = ANGULAR_VELOCITY
            
            elif self.totalTime < self.firstStopTime:                
                self.get_logger().info(f'pausing')
                self.targetAngularVelocity = 0.0
                
            elif self.totalTime < self.rightSweepTime:
                self.get_logger().info(f'sweeping clockwise')
                self.targetAngularVelocity = -1 * ANGULAR_VELOCITY
            
            elif self.totalTime < self.secondStopTime:                
                self.get_logger().info(f'pausing')
                self.targetAngularVelocity = 0.0

            #if time is larger than right and left sweep times, but also smaller than the total sweep time, return to your original position
            elif self.totalTime < self.totalSweepTime:
                self.get_logger().info(f'returning to initial angle')
                self.targetAngularVelocity = ANGULAR_VELOCITY           

    def end_recovery(self):
        self.targetAngularVelocity = 0.0
        self.targetLinearVelocity = 0.0
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_Twist.publish(msg)
        self.state = "noPublishing"
        self.get_logger().info(f'recovery ended')
        response = self.send_request(False)

    def begin_recovery(self):
        self.state = "beginRecovery"
        if self.ultrasoundReading >= THRESHOLD_DISTANCE:
            self.targetAngularVelocity = 0.0
            #updates time to stop backing up
            self.backupEndTime = self.totalTime + BACKUP_TIME
            self.state = "backup"

    #this tests if there is an object closer than 40 centimeters, in which case it calls the sweeping method
        else:
            self.targetLinearVelocity = 0.0
            initialTime = self.totalTime
            
            #updates times to do different things while sweeping
            self.leftSweepTime = initialTime + (SWEEP_TIME / 4) #needs one quarter of the time to go one direction
            self.firstStopTime = self.leftSweepTime + SWEEP_PAUSE_TIME #time to pause after sweeping left
            self.rightSweepTime = self.firstStopTime + (SWEEP_TIME / 2) #needs half the time to go back the other direction to the starting position then go the opposite direction
            self.secondStopTime = self.rightSweepTime + SWEEP_PAUSE_TIME #time to pause after sweeping right
            self.totalSweepTime = initialTime + SWEEP_TIME + (2 * SWEEP_PAUSE_TIME) #total time to do the whole sweep including pauses in between
            self.state = "startSweep"

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
