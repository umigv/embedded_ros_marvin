import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL

WHEEL_BASE = 0.77
WHEEL_DIAMETER = 0.312928
PI = 3.14159265359
VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0 / 3.0
LEFT_POLARITY = 1
RIGHT_POLARITY = -1
ESTOP_FILE_PATH = "/tmp/estop_value.txt"


class DualODriveController(Node):
    def __init__(self):
        super().__init__('dual_odrive_controller')

        #serial may need to be converted to hex?
        self.odrv0 = odrive.find_any(serial_number="3972354E3231") 
        self.odrv1 = odrive.find_any(serial_number="396F35573231")

        self.calibrate_motor()

        self.subscription = self.create_subscription(
            Twist,'merge_vel',self.cmd_vel_callback,10)

        self.publisher = self.create_publisher(Twist, 'enc_vel', 10)

        self.timer = self.create_timer(0.1, self.publish_enc_vel)

    def calibrate_motor(self):

        '''self.get_logger().info("Calibrating...")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while (self.odrv0.axis0.current_state != AXIS_STATE_IDLE):
            pass
        self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while (self.odrv1.axis0.current_state != AXIS_STATE_IDLE):
            pass
        self.get_logger().info("Calibration complete")
        '''

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def cmd_vel_callback(self, msg):
        estop_value = 1  # Default value
        try:
            with open(ESTOP_FILE_PATH, 'r') as f:
                estop_value = int(f.read().strip())
        except:
            estop_value = 1

        linear = msg.linear.x
        angular = msg.angular.z
        left_vel = LEFT_POLARITY * (linear - WHEEL_BASE * angular / 2.0)* VEL_TO_RPS
        right_vel = RIGHT_POLARITY * (linear + WHEEL_BASE * angular / 2.0)* VEL_TO_RPS

        if estop_value == 0:
            self.odrv0.axis0.controller.input_vel = 0
            self.odrv1.axis0.controller.input_vel = 0
        else:
            self.odrv0.axis0.controller.input_vel = left_vel
            self.odrv1.axis0.controller.input_vel = right_vel


    def publish_enc_vel(self):
        enc_vel_left = LEFT_POLARITY * self.odrv0.axis0.vel_estimate / VEL_TO_RPS 
        enc_vel_right = RIGHT_POLARITY * self.odrv1.axis0.vel_estimate / VEL_TO_RPS
        msg = Twist()
        msg.linear.x = (enc_vel_left + enc_vel_right) / 2
        msg.angular.z = (enc_vel_right - enc_vel_left) / WHEEL_BASE
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
