import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial


class EstopSerialNode(Node):
    def __init__(self):
        super().__init__("estop_serial_node")

        # --- Parameters ---
        self.declare_parameter("serial_device", "/dev/ESP32_LORA")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("serial_timeout_s", 0.5)

        self.serial_device = str(self.get_parameter("serial_device").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout_s = float(self.get_parameter("serial_timeout_s").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_period_s = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 0.1

        # --- E-stop state ---
        self.estop_active = True  # default to STOP (fail-safe)
        self.last_serial_msg_time = None

        # --- Serial ---
        self.serial_port = None

        # --- Logging helpers (avoid spam) ---
        self.last_reported_state = None
        self.reported_timeout = False

        # --- ROS interfaces ---
        self.publisher = self.create_publisher(Bool, "estop", 10)
        self.timer = self.create_timer(self.publish_period_s, self.update)

        self.connect_serial()

    def connect_serial(self):
        """Attempt to connect to ESP32 over serial."""
        if self.serial_port and self.serial_port.is_open:
            return

        try:
            resolved = os.path.realpath(self.serial_device)
            self.serial_port = serial.Serial(resolved, self.baudrate, timeout=0.0)
            self.get_logger().info(f"Connected to {resolved}")
        except serial.SerialException as e:
            self.serial_port = None
            self.set_estop_state(True, f"Serial unavailable: {e}")

    def update(self):
        """Main loop: read serial, enforce timeout, publish state."""
        self.read_serial_messages()
        self.enforce_timeout()
        self.publisher.publish(Bool(data=self.estop_active))

    def read_serial_messages(self):
        """Read all available serial messages from ESP32."""
        if not self.serial_port or not self.serial_port.is_open:
            self.connect_serial()
            return

        try:
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode(errors="ignore").strip().upper()

                if not line:
                    continue

                # update heartbeat time
                self.last_serial_msg_time = time.monotonic()
                self.reported_timeout = False

                if line == "RUN":
                    self.set_estop_state(False, "Received RUN")
                elif line == "STOP":
                    self.set_estop_state(True, "Received STOP")
                else:
                    self.get_logger().warning(f"Unknown message: {line}")

        except serial.SerialException as e:
            self.close_serial()
            self.set_estop_state(True, f"Serial read failure: {e}")

    def enforce_timeout(self):
        """Force STOP if messages stop arriving."""
        if self.last_serial_msg_time is None:
            # never received anything -> stay stopped
            if not self.reported_timeout:
                self.set_estop_state(True, "Waiting for first message")
                self.reported_timeout = True
            return

        elapsed = time.monotonic() - self.last_serial_msg_time

        if elapsed > self.serial_timeout_s:
            if not self.reported_timeout:
                self.set_estop_state(True, f"Timeout: {elapsed:.2f}s")
                self.reported_timeout = True

    def set_estop_state(self, active, reason):
        """Update estop state and log only on change."""
        if self.last_reported_state != active:
            state = "STOP" if active else "RUN"
            self.get_logger().info(f"E-stop -> {state}: {reason}")
            self.last_reported_state = active

        self.estop_active = active

    def close_serial(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstopSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
