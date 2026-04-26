import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import os


ESTOP_FILE_PATH = "/tmp/estop_value.txt"
TELEOP_TIMEOUT_SEC = 1.0
TICK_PERIOD_SEC = 0.1

LED_ESTOP = 6
LED_TELEOP = 1
LED_STATE = {
    "normal": 2,
    "no_mans_land": 3,
    "recovery": 4,
}


class LEDSubscriber(Node):
    def __init__(self):
        super().__init__('LED_subscriber')

        self.create_subscription(Twist, 'teleop_cmd_vel', self.teleop_callback, 10)
        self.create_subscription(String, 'state', self.state_callback, 10)

        self.last_teleop_time = None
        self.current_state = None
        self.last_sent = None

        self.serial_symlink = "/dev/LED_Arduino"
        try:
            self.serial_port_path = os.path.realpath(self.serial_symlink)
            self.serial_port = serial.Serial(self.serial_port_path, baudrate=9600, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_symlink} → {self.serial_port_path}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial_port = None

        self.create_timer(TICK_PERIOD_SEC, self.tick)

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now()

    def state_callback(self, msg):
        self.current_state = msg.data

    def is_estopped(self) -> bool:
        try:
            with open(ESTOP_FILE_PATH, "r") as f:
                return f.read().strip() == "1"
        except Exception:
            return False

    def teleop_active(self) -> bool:
        if self.last_teleop_time is None:
            return False
        elapsed = (self.get_clock().now() - self.last_teleop_time).nanoseconds / 1e9
        return elapsed <= TELEOP_TIMEOUT_SEC

    def tick(self):
        if self.is_estopped():
            value = LED_ESTOP
        elif self.teleop_active():
            value = LED_TELEOP
        elif self.current_state in LED_STATE:
            value = LED_STATE[self.current_state]
        else:
            return

        self.send_to_arduino(value)

    def send_to_arduino(self, value: int):
        if not (self.serial_port and self.serial_port.is_open):
            return
        try:
            self.serial_port.write(f"{value}\n".encode())
            if value != self.last_sent:
                self.get_logger().info(f"Sent to Arduino: {value}")
                self.last_sent = value
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    led_subscriber = LEDSubscriber()
    try:
        rclpy.spin(led_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        led_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
