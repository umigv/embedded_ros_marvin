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
class OdriveDriverConfig:
    """Configuration for the ODrive motor driver node.

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
    timestamp_delay_s: float = 0.0
    frame_id: str = "base_link"
    estop_file_path: Path = Path("/tmp/estop_value.txt")

    def __post_init__(self) -> None:
        if self.sample_time_s <= 0:
            raise ValueError("OdriveDriverConfig: sample_time_s must be > 0")
        if self.timestamp_delay_s < 0:
            raise ValueError("OdriveDriverConfig: timestamp_delay_s must be >= 0")

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
