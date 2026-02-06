import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
)

# Robot Parameters
WHEEL_BASE = 0.77  # Distance between wheels (meters)
WHEEL_DIAMETER = 0.192  # Wheel diameter (meters)
PI = 3.14159265359
VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0 / 3.0
LEFT_POLARITY = -1
RIGHT_POLARITY = 1
ESTOP_FILE_PATH = "/tmp/estop_value.txt"
ENCODER_COUNTS_PER_REV = 42  # Encoder resolution (counts per revolution)
SAMPLE_TIME = 0.02  # Time interval for updates (seconds)

# Compute Covariance
circumference = WHEEL_DIAMETER * PI  # meters
distance_per_count = circumference / ENCODER_COUNTS_PER_REV  # meters per count

'''m/s uncertainty for each wheel (assuming no correlation between two wheels,
also assuming encoder deviation of 1 count)'''
vel_std_dev = distance_per_count / SAMPLE_TIME  

'''Variance for linear velocity. For v_x = (v_r + v_l)/2, variance is wheel_variance/2'''
vel_variance = 0.5 * (vel_std_dev ** 2)

'''Variance for angular velocity. For w_z = (v_r - v_l)/L, variance is 2*wheel_variance/L^2'''
ang_variance = 2 * (vel_std_dev ** 2) / (WHEEL_BASE ** 2)

# Covariance Matrix
COVARIANCE_MATRIX = [0.0] * 36
COVARIANCE_MATRIX[0] = vel_variance  # Linear velocity variance
COVARIANCE_MATRIX[35] = ang_variance  # Angular velocity variance


class DualODriveController(Node):
    def __init__(self):
        super().__init__('dual_odrive_controller')

        self.odrv0 = odrive.find_any(serial_number="395934763331")
        self.odrv1 = odrive.find_any(serial_number="384934743539")

        self.motor_setup()

        self.odrive_left = odrive.find_any(serial_number="395934763331")
        self.initialize_odrive(self.odrive_left)

        self.odrive_right = odrive.find_any(serial_number="384934743539")
        self.initialize_odrive(self.odrive_right)

        self.subscription = self.create_subscription(Twist, '/joy_cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, '/enc_vel', 10)
        self.timer = self.create_timer(self.config.sample_time_s, self.publish_enc_vel)

    def cmd_vel_callback(self, msg):
        if not self.is_robot_enabled():
            self.set_motor_rps(0.0, 0.0)
            return
        
        left_rps, right_rps = self.config.twist_to_motor_rps(msg.linear.x, msg.angular.z)
        self.set_motor_rps(left_rps, right_rps)

    def publish_enc_vel(self):
        left_motor_rps, right_motor_rps = self.get_motor_rps()
        linear_mps, angular_radps = self.config.motor_rps_to_twist(left_motor_rps, right_motor_rps)

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.twist.linear.x = linear_mps
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_radps

        msg.twist.covariance = self.config.twist_covariance(linear_mps, angular_radps)

        self.publisher.publish(msg)

    def initialize_odrive(self, odrive) -> None:
        odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def set_motor_rps(self, left_motor_rps: float, right_motor_rps: float) -> None:
        self.odrive_left.axis0.controller.input_vel = left_motor_rps * self.config.left_polarity
        self.odrive_right.axis0.controller.input_vel = right_motor_rps * self.config.right_polarity

    def get_motor_rps(self) -> tuple[float, float]:
        left_motor_rps = self.odrive_left.axis0.vel_estimate * self.config.left_polarity
        right_motor_rps = self.odrive_right.axis0.vel_estimate * self.config.right_polarity
        return left_motor_rps, right_motor_rps

    def is_robot_enabled(self) -> bool:
        try:
            with open(self.config.estop_file_path, "r") as f:
                return f.read().strip() == "1" # yes, e-stop disabled is 1 in the file
        except Exception:
            return True # if the e-stop file doesn's exist / is corrupted we assume e-stop is off

def main(args=None):
    rclpy.init(args=args)
    node = DualODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
