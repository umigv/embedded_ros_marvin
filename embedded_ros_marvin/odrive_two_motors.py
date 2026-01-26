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
MPS_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0 / 3.0
RPS_TO_MPS = 1.0 / MPS_TO_RPS
LEFT_POLARITY = 1
RIGHT_POLARITY = -1
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

        self.odrive_left = odrive.find_any(serial_number="3972354E3231")
        self.initialize_odrive(self.odrive_left)

        self.odrive_right = odrive.find_any(serial_number="396F35573231")
        self.initialize_odrive(self.odrive_right)

        self.subscription = self.create_subscription(Twist, '/joy_cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, '/enc_vel', 10)
        self.timer = self.create_timer(SAMPLE_TIME, self.publish_enc_vel)

    def cmd_vel_callback(self, msg):
        if not self.is_robot_enabled():
            self.set_wheel_rps(0, 0)
            return
        
        left_rps, right_rps = self.twist_to_wheel(msg.linear.x, msg.angular.z)
        self.set_wheel_rps(left_rps, right_rps)

    def publish_enc_vel(self):
        left_rps, right_rps = self.get_wheel_rps()
        linear_mps, angular_radps = self.wheel_to_twist(left_rps, right_rps)

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.twist.linear.x = linear_mps
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_radps
        msg.twist.covariance = COVARIANCE_MATRIX

        self.publisher.publish(msg)

    def initialize_odrive(self, odrive) -> None:
        odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def set_wheel_rps(self, left_rps: float, right_rps: float) -> None:
        self.odrive_left.axis0.controller.input_vel = left_rps * LEFT_POLARITY
        self.odrive_right.axis0.controller.input_vel = right_rps * RIGHT_POLARITY

    def get_wheel_rps(self) -> tuple[float, float]:
        left_rps = self.odrive_left.axis0.vel_estimate * LEFT_POLARITY
        right_rps = self.odrive_right.axis0.vel_estimate * RIGHT_POLARITY
        return left_rps, right_rps

    def wheel_to_twist(self, left_rps: float, right_rps: float) -> tuple[float, float]:
        left_vel = left_rps * RPS_TO_MPS
        right_vel = right_rps * RPS_TO_MPS

        linear_mps = (left_vel + right_vel) / 2.0
        angular_radps = (right_vel - left_vel) / WHEEL_BASE
        return linear_mps, angular_radps

    def twist_to_wheel(self, linear_mps: float, angular_radps: float) -> tuple[float, float]:
        left_vel = linear_mps - (WHEEL_BASE * angular_radps) / 2.0
        right_vel = linear_mps + (WHEEL_BASE * angular_radps) / 2.0

        left_rps = left_vel * MPS_TO_RPS
        right_rps = right_vel * MPS_TO_RPS
        return left_rps, right_rps

    def is_robot_enabled(self) -> bool:
        try:
            with open(ESTOP_FILE_PATH, "r") as f:
                return f.read().strip() == "1"
        except Exception:
            return True

def main(args=None):
    rclpy.init(args=args)
    node = DualODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
