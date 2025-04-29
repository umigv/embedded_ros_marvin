import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from infra_interfaces.action import FollowPath  # Custom action
import math
import time


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lookahead')
        self.get_logger().info('Pure Pursuit Node started.')

        # Parameters
        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.4
        self.lookahead_distance = 0.15
        self.goal_tolerance = 0.2
        self.visited = -1  # last node visited

        # State
        self.path = []
        self.pose = None
        self.reached_goal = False

        # Use ReentrantCallbackGroup for concurrency
        self.cb_group = ReentrantCallbackGroup()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/joy_cmd_vel', 10)

        # Path publisher for RViz debugging
        self.path_pub = self.create_publisher(Path, '/debug_path', 10)
        self.create_timer(1.0, self.publish_path, callback_group=self.cb_group)

        self.action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received a new path from action client.')
        self.path = [(p.x, p.y) for p in goal_handle.request.path]
        self.reached_goal = False
        self.visited = -1
        
        while not self.reached_goal and rclpy.ok():
            time.sleep(0.05)

        self.path = []
        goal_handle.succeed()
        result = FollowPath.Result()
        return result

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(ori)
        self.pose = (pos.x, pos.y, yaw)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_lookahead_point(self):
        if self.pose is None or not self.path:
            self.get_logger().info('Odom or Path not available')
            return None

        x, y, yaw = self.pose

        # Check if the goal is reached
        goal_x, goal_y = self.path[-1]
        goal_dx = goal_x - x
        goal_dy = goal_y - y
        goal_dist = math.hypot(goal_dx, goal_dy)
        if goal_dist < self.goal_tolerance:
            self.get_logger().info('REACHED GOAL')
            self.reached_goal = True
            return None

        # Search for lookahead point
        for i, (gx, gy) in enumerate(self.path):
            if i <= self.visited:
                continue

            dx = gx - x
            dy = gy - y

            # Transform to robot frame
            local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            dist = math.hypot(local_x, local_y)

            if local_x > 0.0 and dist >= self.lookahead_distance:
                self.visited = i
                return local_x, local_y

        self.get_logger().info('AHHHHHHHH (cannot find lookahead point)')
        return None

    def control_loop(self):
        local_point = self.find_lookahead_point()

        if local_point is None:
            self.cmd_pub.publish(Twist())
            return

        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)
        dist = math.hypot(local_x, local_y)

        linear = min(self.max_linear_speed, dist)
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, linear * curvature))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def publish_path(self):

        if not self.path:
            return

        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom" #allows visualization in rviz

        for x, y in self.path:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
