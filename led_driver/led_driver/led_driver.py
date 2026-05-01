import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from .led_driver_config import LedDriverConfig


class LedDriver(Node):
    LED_ESTOP = 6  # Flashing red
    LED_TELEOP = 1  # Solid blue
    LED_STATE = {
        "normal": 2,  # Flashing blue
        "no_mans_land": 3,  # Flashing green
        "recovery": 4,  # Flashing yellow
    }
    LED_UNKNOWN = 9  # Rainbow

    def __init__(self) -> None:
        super().__init__('led_driver')

        self.config = LedDriverConfig()

        self.create_subscription(Twist, 'teleop_cmd_vel', self.teleop_callback, 10)
        self.create_subscription(String, 'state', self.state_callback, 10)

        self.last_teleop_time: Time | None = None
        self.state: str | None = None
        self.last_sent: int | None = None
        self.serial: serial.Serial | None = None
        
        try:
            self.serial = serial.Serial(str(self.config.serial_port), baudrate=self.config.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.config.serial_port}")
            self.wait_for_ready()
        except (serial.SerialException, RuntimeError) as e:
            self.get_logger().error(f"Failed to connect to {self.config.serial_port}: {e}")
            # We don't call rclpy.shutdown() here because it causes a deadlock in humble
            # https://github.com/ros2/rclpy/issues/1646
            raise SystemExit(1) from None

        self.create_timer(self.config.update_period_s, self.update)

    def wait_for_ready(self) -> None:
        assert self.serial is not None

        deadline = time.monotonic() + self.config.ready_timeout_s
        while time.monotonic() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if line == "READY":
                self.get_logger().info("READY received")
                return
        
        raise RuntimeError(f"Did not receive READY within {self.config.ready_timeout_s}s")

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()

    def state_callback(self, msg):
        self.state = msg.data

    def is_robot_enabled(self) -> bool:
        try:
            with open(self.config.estop_file_path, "r") as f:
                return f.read().strip() != "1" # only "1" stops the robot, everything else is enabled
        except Exception:
            return True # if the e-stop file doesn't exist / is corrupted we assume e-stop is off

    def is_teleop_active(self) -> bool:
        if self.last_teleop_time is None:
            return False
        
        elapsed = (self.get_clock().now() - self.last_teleop_time).nanoseconds / 1e9
        return elapsed <= self.config.teleop_timeout_s

    def update(self) -> None:
        if not self.is_robot_enabled():
            value = self.LED_ESTOP
        elif self.is_teleop_active():
            value = self.LED_TELEOP
        elif self.state in self.LED_STATE:
            value = self.LED_STATE[self.state]
        else:
            value = self.LED_UNKNOWN

        self.send_led_value(value)

    def send_led_value(self, value: int):
        assert self.serial is not None

        if not self.serial.is_open:
            return

        if value == self.last_sent:
            return

        try:
            self.serial.write(f"{value}\n".encode())
            self.get_logger().info(f"Sent: {value}")
            self.last_sent = value
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LedDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
