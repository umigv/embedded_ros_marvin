import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class EncOdomPublisher(Node):
    def __init__(self):
        super().__init__('enc_odom_publisher')

        # Declare and get the parameter
        self.declare_parameter('use_enc_tf', True)
        self.use_enc_tf = self.get_parameter('use_enc_tf').value

        # Initialize pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time for odometry calculation
        self.prev_time = self.get_clock().now()

        # Subscriber for encoder velocity
        self.subscription = self.create_subscription(
            Twist,
            'enc_vel',
            self.enc_vel_callback,
            10
        )

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom/encoder', 10)

        # TF broadcaster (only if use_enc_tf is True)
        if self.use_enc_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

    def enc_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert to seconds
        self.prev_time = current_time

        # Extract velocities from message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Update pose using differential drive kinematics
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        self.theta += angular_vel * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry message
        self.publish_odom(linear_vel, angular_vel, current_time)

    def publish_odom(self, linear_vel, angular_vel, current_time):
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Twist (velocity)
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        # Publish odometry
        self.odom_publisher.publish(odom_msg)

        # Publish TF if enabled
        if self.use_enc_tf:
            self.publish_tf(current_time)

    def publish_tf(self, current_time):
        # Create transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

