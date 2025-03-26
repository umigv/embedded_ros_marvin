import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.lookahead_distance = 0.5
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        self.goal_tolerance = 0.1

        # State
        self.goal = None
        self.pose = None

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/joy_cmd_vel', 10)

        # Timer
        self.create_timer(0.05, self.control_loop)

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(ori)
        self.pose = (pos.x, pos.y, yaw)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.pose is None or self.goal is None:
            return

        x, y, yaw = self.pose
        goal_x, goal_y = self.goal

        dx = goal_x - x
        dy = goal_y - y
        dist_to_goal = distance((x, y), (goal_x, goal_y))

        # Stop if close enough
        if dist_to_goal < self.goal_tolerance:
            self.cmd_pub.publish(Twist())  # Stop
            self.get_logger().info('Goal reached.')
            return

        # Transform goal to robot's frame
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # Prevent division by zero
        if local_x <= 0.001:
            return

        # Compute curvature and steering
        curvature = 2 * local_y / (local_x**2 + local_y**2)
        linear = min(self.max_linear_speed, dist_to_goal)
        angular = linear * curvature
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        # Publish command
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()