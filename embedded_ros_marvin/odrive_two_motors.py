import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
)
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class DriveConfig:
    # Geometry / drivetrain
    track_width_m: float = 0.724
    wheel_diameter_m: float = 0.181356
    gear_ratio: float = 98.0 / 3.0

    # Polarity (motor-native <-> robot-forward convention)
    left_polarity: int = -1
    right_polarity: int = 1

    # Sampling / encoder
    encoder_counts_per_motor_rev: int = 42
    sample_time_s: float = 0.02

    # E-stop
    estop_file_path: str = "/tmp/estop_value.txt"

    # Dynamic covariance model (variance = floor + gain * f(speed)^2)
    linear_variance_gain: float = 0.004
    min_linear_variance: float = 1e-6
    angular_variance_gain: float = 0.003
    min_angular_variance: float = 1e-6

    @property
    def wheel_circumference_m(self) -> float:
        return self.wheel_diameter_m * math.pi

    @property
    def motor_rps_per_wheel_mps(self) -> float:
        return self.gear_ratio / self.wheel_circumference_m

    @property
    def wheel_mps_per_motor_rps(self) -> float:
        return 1.0 / self.motor_rps_per_wheel_mps

    @property
    def wheel_velocity_stddev_mps(self) -> float:
        encoder_counts_per_wheel_rev = float(self.encoder_counts_per_motor_rev) * float(self.gear_ratio)
        distance_per_count = self.wheel_circumference_m / encoder_counts_per_wheel_rev
        return distance_per_count / self.sample_time_s

    @property
    def linear_variance_static(self) -> float:
        return 0.5 * (self.wheel_velocity_stddev_mps ** 2)

    @property
    def angular_variance_static(self) -> float:
        return 2.0 * (self.wheel_velocity_stddev_mps ** 2) / (self.track_width_m ** 2)

    def twist_covariance(self, linear_mps: float, angular_radps: float) -> list[float]:
        linear_variance_dynamic = self.linear_variance_gain * (linear_mps ** 2)
        angular_variance_dynamic = self.angular_variance_gain * (angular_radps ** 2)

        linear_variance = max(self.min_linear_variance, self.linear_variance_static + linear_variance_dynamic)
        angular_variance = max(self.min_angular_variance, self.angular_variance_static + angular_variance_dynamic)

        cov = [0.0] * 36
        cov[0] = linear_variance
        cov[35] = angular_variance
        return cov
    
    def motor_rps_to_twist(self, left_motor_rps: float, right_motor_rps: float) -> tuple[float, float]:
        left_wheel_mps = left_motor_rps * self.wheel_mps_per_motor_rps
        right_wheel_mps = right_motor_rps * self.wheel_mps_per_motor_rps

        linear_mps = (left_wheel_mps + right_wheel_mps) / 2.0
        angular_radps = (right_wheel_mps - left_wheel_mps) / self.track_width_m
        return linear_mps, angular_radps

    def twist_to_motor_rps(self, linear_mps: float, angular_radps: float) -> tuple[float, float]:
        left_wheel_mps = linear_mps - (self.track_width_m * angular_radps) / 2.0
        right_wheel_mps = linear_mps + (self.track_width_m * angular_radps) / 2.0

        left_motor_rps = left_wheel_mps * self.motor_rps_per_wheel_mps
        right_motor_rps = right_wheel_mps * self.motor_rps_per_wheel_mps
        return left_motor_rps, right_motor_rps

class DualODriveController(Node):
    def __init__(self):
        super().__init__('dual_odrive_controller')

        self.config = DriveConfig()

        self.subscription = self.create_subscription(
            Twist, 'joy_cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for encoder velocities with covariance
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'enc_vel', 10)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'enc_vel', 10)
        self.timer = self.create_timer(self.config.sample_time_s, self.publish_enc_vel)

    def cmd_vel_callback(self, msg):
        estop_value = 1  # Default value (assume not stopped)
        try:
            with open(ESTOP_FILE_PATH, 'r') as f:
                estop_value = int(f.read().strip())
        except Exception:
            estop_value = 1  # If file not found, assume no stop condition

        # Compute left and right wheel velocities
        linear = msg.linear.x
        angular = msg.angular.z
        left_vel = LEFT_POLARITY * (linear - WHEEL_BASE * angular / 2.0) * VEL_TO_RPS
        right_vel = RIGHT_POLARITY * (linear + WHEEL_BASE * angular / 2.0) * VEL_TO_RPS

        # Apply emergency stop logic
        if estop_value == 0:
            self.odrv0.axis0.controller.input_vel = 0
            self.odrv1.axis0.controller.input_vel = 0
        else:
            self.odrv0.axis0.controller.input_vel = left_vel
            self.odrv1.axis0.controller.input_vel = right_vel

    def publish_enc_vel(self):
        """ Publishes estimated encoder velocity with covariance. """
        # Compute estimated velocities from each wheel
        enc_vel_left = LEFT_POLARITY * self.odrv0.axis0.vel_estimate / VEL_TO_RPS 
        enc_vel_right = RIGHT_POLARITY * self.odrv1.axis0.vel_estimate / VEL_TO_RPS
        linear_vel = (enc_vel_left + enc_vel_right) / 2.0
        angular_vel = (enc_vel_right - enc_vel_left) / WHEEL_BASE

        # Create a TwistWithCovarianceStamped message
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Frame in which velocities are measured

        # Fill in the twist data
        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_vel

        # Use precomputed covariance matrix
        msg.twist.covariance = COVARIANCE_MATRIX

        # Publish the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
