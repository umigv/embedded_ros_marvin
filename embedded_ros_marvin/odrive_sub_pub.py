import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL


class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        
        #self.odrv0 = odrive.find_any()

        self.odrv0 = odrive.find_any(serial_number="3972354E3231")
        #self.odrv0 = odrive.find_any(serial_number="393F35423231")

        self.calibrate_motor()

        # Create subscriber 
        self.subscription = self.create_subscription(
            Twist,'cmd_vel',self.cmd_vel_callback,10)

        # Create publisher
        self.publisher = self.create_publisher(Twist, 'enc_vel', 10)

        # Create a timer to publish enc_vel at 10hz
        self.timer = self.create_timer(0.1, self.publish_enc_vel)

    def calibrate_motor(self):
        self.get_logger().info("Calibrating...")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass
        self.get_logger().info("Calibration complete")

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    def cmd_vel_callback(self, msg):
        self.odrv0.axis0.controller.input_vel = msg.linear.x

    def publish_enc_vel(self):
        enc_vel = self.odrv0.axis0.vel_estimate
        msg = Twist()
        msg.linear.x = enc_vel
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
