import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from rclpy.executors import ExternalShutdownException
import math
from std_srvs.srv import SetBool
import sys

recovery_executable = None

class RecoveryExecutable(Node):
    def __init__(self):
        super().__init__('recovery_executable')
        self.publisher_Twist = self.create_publisher(Twist, 'recovery_cmd_vel', 10)
        #self.publisher_Boolean = self.create_publisher(Bool, 'recoveryOngoingTopic', 10)
        timer_period = 0.5  # seconds
        self.velocity_control_period = 0.5 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(self.velocity_control_period, self.set_velocity_from_error)
        #self.subscription = self.create_subscription(Bool, 'recoveryOngoingTopic', self.listener_callback, 10)
        self.subscription = self.create_subscription(String, 'state', self.listener_callback, 10)

        self.cli = self.create_client(SetBool, 'state/set_recovery')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

        self.timeElapsed = 0.0
        self.ultraSoundReadingFloat = 2000.0
        self.targetLinearVelocity = 0.0 #m/s???
        self.targetAngularVelocity = 0.0 #rad/s???
        self.setTargetLinearVelocity = False
        self.targetPosition = 0.30 #meters
        self.distanceTraveled = 0.0
        self.proportional = 0.333
        self.recoveryOngoing = False
        self.beginSweeping = False
        self.radiansTravelled = 0.0
        self.turnRight = False
        self.sweepBegan = False
        
        ultrasoundTimerPeriod = 0.05
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port")
            self.arduino = None
        self.timer3 = self.create_timer(ultrasoundTimerPeriod, self.updateArduinoReading)

    #This gets in the arduino reading and updates the self.ultraSoundReadingFloat, which is where we use the ultrasound readigns in the rest of the code
    def updateArduinoReading(self):
        if self.arduino is None:
            return

        if self.arduino.in_waiting > 0:
            packet = self.arduino.readline()
            ultraSoundReadingString = packet.decode('utf-8', errors='ignore').rstrip('\n')
            try:
                self.ultraSoundReadingFloat = float(ultraSoundReadingString)
                #self.get_logger().info("setting float value")
            except:
                self.get_logger().info("not reading ultrasound values")

    def listener_callback(self, msg): #Note from Hannah & Ash: How does msg variable work, and does it conflict with msg var in twist?
        #self.get_logger().info(f'recoveryOngoing: {bool(msg.data)}')
        if msg.data == 'recovery':
            self.recoveryOngoing = True

    #This funciton is called periodically. It actually publishes the velocity messages and has the logic of whether to call the function that just drives teh robot backwards or call the sweep function
    def timer_callback(self):
        #self.get_logger().info(f'calling timer callback')
        self.get_logger().info(f'ultrasoundreading: {self.ultraSoundReadingFloat}')

    #this is where the velocity messages are actually published
        msg = Twist()
        msg.linear.x = -self.targetLinearVelocity
        msg.angular.z = self.targetAngularVelocity
        #self.get_logger().info(str(msg))
        if self.recoveryOngoing == True:
            self.publisher_Twist.publish(msg)
        #self.get_logger().info(f'ultrasoundreading: {self.ultraSoundReadingFloat}')
        self.timeElapsed += 0.5

    #this tests if there is an object closer than 40 centimeters to the robot, and calls the method to back up if there isn't anything
        if self.ultraSoundReadingFloat >= 40.0 and self.recoveryOngoing == True:
            self.setTargetLinearVelocity = True
            self.targetAngularVelocity = 0.0
            self.beginSweeping = False
            #self.get_logger().info("target velocity is true")

    #this tests if there is an object closer than 40 centimeters, in which case it calls the sweeping method
        if self.ultraSoundReadingFloat < 40.0 and self.recoveryOngoing == True:
            self.setTargetLinearVelocity = False
            self.targetLinearVelocity = 0.0
            self.beginSweeping = True
            self.sweep_left_and_right()
#        if self.ultraSoundReadingFloat <= 40.0:
#            self.set_conditional_velocity(self.ultraSoundReadingFloat)

    #this function sets the velocity to have the robot back up. The velocity starts large, then decreases as the robot gets closer to its target
    #currently, it is set to backup 0.30 meters
    def set_velocity_from_error(self):

        if self.setTargetLinearVelocity == True and self.recoveryOngoing == True:
            self.get_logger().info("backing up")
            self.distanceTraveled = self.distanceTraveled + (self.velocity_control_period * self.targetLinearVelocity)
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
        if self.beginSweeping == True and self.recoveryOngoing == True:
            self.sweepBegan = True
            self.get_logger().info(f'sweeping')
            #sweeps left pi/2 radians or 90 degrees one way at a speed of pi/9 radians per second
            if self.setTargetLinearVelocity == False and self.radiansTravelled < math.pi / 2 and self.turnRight == False:
                self.targetAngularVelocity = math.pi / 9 
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * 0.5)
            elif self.radiansTravelled >= 0.349066: #sweeps 20 degrees one direction
                self.turnRight = True
            #sweeps the other direction 40 degrees
            if self.setTargetLinearVelocity == False and self.radiansTravelled > -0.349066 and self.turnRight == True:
                self.targetAngularVelocity = -1 * math.pi / 9 
                self.radiansTravelled = self.radiansTravelled + (self.targetAngularVelocity * 0.5)
            elif self.radiansTravelled <= -0.349066:
                self.end_recovery()
        elif self.beginSweeping == False and self.sweepBegan == True:
            self.sweepBegan = False
            self.radiansTravelled = 0.0
    
    def end_recovery(self):
        self.targetAngularVelocity = 0.0
        self.beginSweeping = False
        self.radiansTravelled = 0.0
        self.targetLinearVelocity = 0.0
        self.setTargetLinearVelocity = False
        self.distanceTraveled = 0.0
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_Twist.publish(msg)
        self.recoveryOngoing = False
        # boolmsg = Bool()
        # boolmsg.data = False
        self.get_logger().info(f'recovery Failed')
        response = self.send_request(recovery_executable.recoveryOngoing)
        # self.publisher_Boolean.publish(boolmsg)

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