import os
from dataclasses import dataclass
from pathlib import Path

import rclpy
import serial
from rclpy.node import Node


@dataclass(frozen=True)
class EstopDriverConfig:
    """Config for EstopDriver.

    Attributes:
        serial_port: Path to the serial device.
        baud_rate: Serial baud rate for communication.
        poll_period_s: Period (s) at which the serial port is polled for new data.
        estop_file_path: Path to the file the latest e-stop state is written to ("1" = stopped).
    """

    serial_port: Path = Path("/dev/estop")
    baud_rate: int = 9600
    poll_period_s: float = 0.05
    estop_file_path: Path = Path("/tmp/estop_value.txt")

    def __post_init__(self) -> None:
        if self.baud_rate <= 0:
            raise ValueError("EstopDriverConfig: baud_rate must be > 0")
        if self.poll_period_s <= 0:
            raise ValueError("EstopDriverConfig: poll_period_s must be > 0")


class EstopDriver(Node):
    def __init__(self) -> None:
        super().__init__('estop_driver')

        self.config = EstopDriverConfig()

        self.current_state: str | None = None
        self.read_buffer: bytes = b""
        self.serial: serial.Serial | None = None

        try:
            self.serial = serial.Serial(str(self.config.serial_port), baudrate=self.config.baud_rate, timeout=0.0)
            self.get_logger().info(f"Connected to {self.config.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.config.serial_port}: {e}")
            # We don't call rclpy.shutdown() here because it causes a deadlock in humble
            # https://github.com/ros2/rclpy/issues/1646
            raise SystemExit(1) from None

        self.create_timer(self.config.poll_period_s, self.poll)

    def poll(self) -> None:
        assert self.serial is not None

        if self.serial.in_waiting <= 0:
            return

        self.read_buffer += self.serial.read(self.serial.in_waiting)
        while b"\n" in self.read_buffer:
            raw, _, self.read_buffer = self.read_buffer.partition(b"\n")
            line = raw.decode("utf-8", errors="ignore").strip()

            if line not in ("0", "1"):
                self.get_logger().warning(f"Unexpected estop line: {line!r}", throttle_duration_sec=2.0)
                continue
            if line == self.current_state:
                continue

            self.write_estop_state(line)
            self.current_state = line
            state_str = "STOP" if line == "1" else "SAFE"
            self.get_logger().info(f"State changed: {line} -> {state_str}")

    def write_estop_state(self, value: str) -> None:
        temp_file_path = self.config.estop_file_path.with_suffix(
            self.config.estop_file_path.suffix + ".tmp"
        )
        
        with open(temp_file_path, "w") as f:
            f.write(value)
            f.flush()
            os.fsync(f.fileno())
        
        # Atomic replace so readers never see a half-written file.
        os.replace(temp_file_path, self.config.estop_file_path)

    def destroy_node(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EstopDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
