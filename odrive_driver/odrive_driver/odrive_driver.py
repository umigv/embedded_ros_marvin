import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistWithCovariance, Twist, Vector3
from std_msgs.msg import Header
import odrive
from odrive.enums import AxisState, ControlMode

from .odrive_driver_config import CovarianceConfig, GeometryConfig, OdriveConfig, OdriveDriverConfig


class OdriveDriver(Node):
    def __init__(self):
        super().__init__('odrive_driver')

        self.config = OdriveDriverConfig(
            left_odrive=OdriveConfig(serial="395534753331"),
            right_odrive=OdriveConfig(serial="384934743539"),
            geometry=GeometryConfig(),
            covariance=CovarianceConfig(),
        )

        self.get_logger().info(f"Finding left ODrive (serial number {self.config.left_odrive.serial})...")
        self.odrive_left = odrive.find_any(serial_number=self.config.left_odrive.serial)
        self.initialize_odrive(self.odrive_left)
        self.get_logger().info("Found left ODrive")
        
        self.get_logger().info(f"Finding right ODrive (serial number {self.config.right_odrive.serial})...")
        self.odrive_right = odrive.find_any(serial_number=self.config.right_odrive.serial)
        self.initialize_odrive(self.odrive_right)
        self.get_logger().info("Found right ODrive")

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'enc_vel', 10)

        self.create_timer(self.config.sample_time_s, self.publish_enc_vel)

    def cmd_vel_callback(self, msg):
        if not self.is_robot_enabled():
            self.get_logger().warning("Robot disabled", throttle_duration_sec=2.0)
            self.set_motor_rps(0.0, 0.0)
            return

        left_rps, right_rps = self.config.twist_to_motor_rps(msg.linear.x, msg.angular.z)
        self.set_motor_rps(left_rps, right_rps)

    def publish_enc_vel(self):
        left_motor_rps, right_motor_rps = self.get_motor_rps()
        linear_mps, angular_radps = self.config.motor_rps_to_twist(left_motor_rps, right_motor_rps)

        # Back-date to compensate for encoder read and processing latency
        timestamp_adjustment = self.get_clock().now() - Duration(nanoseconds=int(self.config.timestamp_delay_s * 1e9))
        self.publisher.publish(TwistWithCovarianceStamped(
            header=Header(stamp=timestamp_adjustment.to_msg(), frame_id=self.config.frame_id),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=linear_mps, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular_radps),
                ),
                covariance=self.config.twist_covariance(linear_mps, angular_radps),
            ),
        ))

    def initialize_odrive(self, odrv) -> None:
        odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

    def set_motor_rps(self, left_motor_rps: float, right_motor_rps: float) -> None:
        self.odrive_left.axis0.controller.input_vel = left_motor_rps
        self.odrive_right.axis0.controller.input_vel = right_motor_rps

    def get_motor_rps(self) -> tuple[float, float]:
        return self.odrive_left.axis0.vel_estimate, self.odrive_right.axis0.vel_estimate

    def is_robot_enabled(self) -> bool:
        try:
            with open(self.config.estop_file_path, "r") as f:
                return f.read().strip() != "1" # only "1" stops the robot, everything else is enabled
        except Exception:
            return True # if the e-stop file doesn't exist / is corrupted we assume e-stop is off

def main() -> None:
    rclpy.init()
    node = OdriveDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
