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
WHEEL_DIAMETER = 0.2032  # Wheel diameter (meters)
PI = 3.14159265359
VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0 / 3.0
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

        self.odrv0 = odrive.find_any(serial_number="3972354E3231")
        self.odrv1 = odrive.find_any(serial_number="396F35573231")

        self.motor_setup()

        self.subscription = self.create_subscription(
            Twist, 'joy_cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for encoder velocities with covariance
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'enc_vel', 10)

        # Timer to periodically publish encoder velocity
        self.timer = self.create_timer(SAMPLE_TIME, self.publish_enc_vel)

    def motor_setup(self):
        """ Set motors to closed-loop velocity control mode. """
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

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
