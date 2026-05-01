import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistWithCovariance, Twist, Vector3
from std_msgs.msg import Header
import odrive
from odrive.enums import AxisState, ControlMode
import math
from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class OdriveConfig:
    """Hardware identification and polarity correction for one ODrive unit.

    Attributes:
        serial: Serial number of the ODrive unit.
        polarity: Sign correction mapping motor-native direction to robot-forward (+1 or -1).
    """

    serial: str
    polarity: int = 1

    def __post_init__(self) -> None:
        if not self.serial:
            raise ValueError("OdriveConfig: serial must be a non-empty string")
        if self.polarity not in (1, -1):
            raise ValueError("OdriveConfig: polarity must be 1 or -1")

@dataclass(frozen=True)
class GeometryConfig:
    """Drivetrain geometry for a differential drive robot.

    Attributes:
        track_width_m: Distance between left and right wheel contact points (m).
        wheel_diameter_m: Diameter of each drive wheel (m).
        gear_ratio: Motor-to-wheel gear ratio (motor revolutions per wheel revolution).
    """

    track_width_m: float = 0.764
    wheel_diameter_m: float = 0.18423
    gear_ratio: float = 170.0 / 9.0

    def __post_init__(self) -> None:
        if self.track_width_m <= 0:
            raise ValueError("GeometryConfig: track_width_m must be > 0")
        if self.wheel_diameter_m <= 0:
            raise ValueError("GeometryConfig: wheel_diameter_m must be > 0")
        if self.gear_ratio <= 0:
            raise ValueError("GeometryConfig: gear_ratio must be > 0")

    @property
    def wheel_circumference_m(self) -> float:
        return self.wheel_diameter_m * math.pi

    @property
    def motor_rps_per_wheel_mps(self) -> float:
        return self.gear_ratio / self.wheel_circumference_m

    @property
    def wheel_mps_per_motor_rps(self) -> float:
        return 1.0 / self.motor_rps_per_wheel_mps

@dataclass(frozen=True)
class CovarianceConfig:
    """Dynamic covariance model for the published twist estimate.

    Variance scales with speed to reflect increased uncertainty at higher velocities:
        linear_variance  = linear_variance_static  + linear_variance_gain  * linear_mps²
        angular_variance = angular_variance_static + angular_variance_gain * 
                           (linear_mps² / track_width_m² + angular_radps²)

    Attributes:
        linear_variance_static: Baseline linear velocity variance, independent of speed (m²/s²).
        linear_variance_gain: Speed-dependent gain on linear velocity variance (m²/s² per (m/s)²).
        angular_variance_static: Baseline angular velocity variance, independent of speed (rad²/s²).
        angular_variance_gain: Speed-dependent gain on angular velocity variance (rad²/s² per (m/s)²).
    """

    linear_variance_static: float = 1e-6
    linear_variance_gain: float = 0.0004
    angular_variance_static: float = 1e-6
    angular_variance_gain: float = 0.0004

    def __post_init__(self) -> None:
        if self.linear_variance_static <= 0:
            raise ValueError("CovarianceConfig: linear_variance_static must be > 0")
        if self.linear_variance_gain < 0:
            raise ValueError("CovarianceConfig: linear_variance_gain must be >= 0")
        if self.angular_variance_static <= 0:
            raise ValueError("CovarianceConfig: angular_variance_static must be > 0")
        if self.angular_variance_gain < 0:
            raise ValueError("CovarianceConfig: angular_variance_gain must be >= 0")

@dataclass(frozen=True)
class DualOdriveConfig:
    """Configuration for the dual ODrive motor controller.

    Attributes:
        left_odrive: Hardware identification and polarity for the left ODrive unit.
        right_odrive: Hardware identification and polarity for the right ODrive unit.
        geometry: Drivetrain geometry.
        covariance: Dynamic covariance model for the published twist estimate.
        sample_time_s: Period of the encoder publish timer (s).
        timestamp_delay_s: Amount subtracted from the publish timestamp to compensate read and processing latency (s).
        frame_id: TF frame ID of the robot base, attached to the published twist header.
        estop_file_path: Path to the e-stop flag file. A value of "1" disables motor output.
    """

    left_odrive: OdriveConfig
    right_odrive: OdriveConfig
    geometry: GeometryConfig
    covariance: CovarianceConfig
    sample_time_s: float = 0.01
    timestamp_delay_s: float = 0.01
    frame_id: str = "base_link"
    estop_file_path: Path = Path("/tmp/estop_value.txt")

    def __post_init__(self) -> None:
        if self.sample_time_s <= 0:
            raise ValueError("DualOdriveConfig: sample_time_s must be > 0")
        if self.timestamp_delay_s < 0:
            raise ValueError("DualOdriveConfig: timestamp_delay_s must be >= 0")

    def twist_covariance(self, linear_mps: float, angular_radps: float) -> list[float]:
        linear_variance_dynamic = self.covariance.linear_variance_gain * (linear_mps ** 2)
        angular_variance_dynamic = self.covariance.angular_variance_gain * (
            linear_mps ** 2 / self.geometry.track_width_m ** 2 + angular_radps ** 2
        )

        linear_variance = self.covariance.linear_variance_static + linear_variance_dynamic
        angular_variance = self.covariance.angular_variance_static + angular_variance_dynamic

        cov = [0.0] * 36
        cov[0] = linear_variance
        cov[35] = angular_variance
        return cov
    
    def motor_rps_to_twist(self, left_motor_rps: float, right_motor_rps: float) -> tuple[float, float]:
        left_wheel_mps = left_motor_rps * self.geometry.wheel_mps_per_motor_rps * self.left_odrive.polarity
        right_wheel_mps = right_motor_rps * self.geometry.wheel_mps_per_motor_rps * self.right_odrive.polarity

        linear_mps = (left_wheel_mps + right_wheel_mps) / 2.0
        angular_radps = (right_wheel_mps - left_wheel_mps) / self.geometry.track_width_m
        return linear_mps, angular_radps

    def twist_to_motor_rps(self, linear_mps: float, angular_radps: float) -> tuple[float, float]:
        left_wheel_mps = linear_mps - (self.geometry.track_width_m * angular_radps) / 2.0
        right_wheel_mps = linear_mps + (self.geometry.track_width_m * angular_radps) / 2.0

        left_motor_rps = left_wheel_mps * self.geometry.motor_rps_per_wheel_mps * self.left_odrive.polarity
        right_motor_rps = right_wheel_mps * self.geometry.motor_rps_per_wheel_mps * self.right_odrive.polarity
        return left_motor_rps, right_motor_rps

class DualODriveController(Node):
    def __init__(self):
        super().__init__('dual_odrive_controller')

        self.config = DualOdriveConfig(
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
    node = DualODriveController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
