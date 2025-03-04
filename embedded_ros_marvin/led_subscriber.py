import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  
import serial
import os


class LEDSubscriber(Node):
    def __init__(self):
        super().__init__('LED_subscriber') 

        # Declare a subscription to 'is_auto' topic, which publishes Int32 messages
        self.subscription = self.create_subscription(
            Int32,
            'is_auto',
            self.is_auto_callback,
            10
        )

        # Use the udev-defined port symlink
        self.serial_symlink = "/dev/LED_Arduino"

        try:
            # Resolve the actual port (e.g., /dev/ttyACM0)
            self.serial_port_path = os.path.realpath(self.serial_symlink)
            
            # Open the serial port
            self.serial_port = serial.Serial(self.serial_port_path, baudrate=9600, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_symlink} → {self.serial_port_path}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial_port = None

    def is_auto_callback(self, msg):
        """Send the received integer to the Arduino over serial."""
        if self.serial_port and self.serial_port.is_open:
            try:
                data_str = f"{msg.data}\n"  # Add newline for proper Arduino parsing
                self.serial_port.write(data_str.encode())
                self.get_logger().info(f"Sent to Arduino: {msg.data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self):
        """Ensure the serial port is properly closed on shutdown."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node and spin
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
