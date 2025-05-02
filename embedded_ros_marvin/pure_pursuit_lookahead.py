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
        self.lookahead_distance = 0.25
        self.goal_tolerance = 0.3
        self.visited = 0  # last node visited

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
        self.visited = 0
        
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
        if self.pose is None or not self.path or len(self.path) < 2:
            self.get_logger().info('Odom or Path not available')
            return None
    
        # Robot's current pose and heading
        robot_x, robot_y, robot_yaw = self.pose
    
        goal_x, goal_y = self.path[-1]
        goal_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if goal_distance < self.goal_tolerance:
            self.get_logger().info('REACHED GOAL')
            self.reached_goal = True
            return None
    
        # Look through each path segment starting from last visited
        for i in range(self.visited, len(self.path) - 1):
            # Get start and end of segment
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]
    
            # Define difference in x and y from start to end of segment
            seg_dx = x2 - x1
            seg_dy = y2 - y1
    
            """
            We want to find a point along the segment from (x1, y1) to (x2, y2) that is
            lookahead_distance away from the robot's current pose (robot_x, robot_y).
    
            Parametric point on the segment:
                x(t) = x1 + t * seg_dx
                y(t) = y1 + t * seg_dy
                with t in [0, 1]
    
            The squared distance to the robot is:
                (x(t) - robot_x)^2 + (y(t) - robot_y)^2 = L^2
    
            Substitute:
                [(x1 - robot_x) + t * seg_dx]^2 + [(y1 - robot_y) + t * seg_dy]^2 = L^2
            
            Expand both square terms:
                (x1 - robot_x)^2 + 2*t*(x1 - robot_x)*seg_dx + t^2*seg_dx^2
              + (y1 - robot_y)^2 + 2*t*(y1 - robot_y)*seg_dy + t^2*seg_dy^2 = L^2
            
            Group like terms:
                t^2 * (seg_dx^2 + seg_dy^2)
              + t * 2 * [(x1 - robot_x)*seg_dx + (y1 - robot_y)*seg_dy]
              + (x1 - robot_x)^2 + (y1 - robot_y)^2 - L^2 = 0
            
            This gives a standard quadratic equation of the form:
                a * t^2 + b * t + c = 0
    
            Where:
                a = seg_dx^2 + seg_dy^2
                b = 2 * ((x1 - robot_x) * seg_dx + (y1 - robot_y) * seg_dy)
                c = (x1 - robot_x)^2 + (y1 - robot_y)^2 - lookahead_distance^2
            """
    
            a = seg_dx**2 + seg_dy**2
            b = 2 * ((x1 - robot_x) * seg_dx + (y1 - robot_y) * seg_dy)
            c = (x1 - robot_x)**2 + (y1 - robot_y)**2 - self.lookahead_distance**2
    
            discriminant = b**2 - 4 * a * c
    
            # No intersection between segment and lookahead circle, or seg_dx and seg_dy both = 0
            if discriminant < 0 or a == 0:
                continue
    
            # Compute possible t values where segment hits lookahead circle
            sqrt_disc = math.sqrt(discriminant)
            t_candidates = [(-b + sqrt_disc) / (2 * a), (-b - sqrt_disc) / (2 * a)]
    
            for t in t_candidates:
                # Only consider t between 0 and 1 (i.e., within the segment)
                if 0.0 <= t <= 1.0:
                    # Interpolated global coordinates at distance = lookahead
                    lookahead_x = x1 + t * seg_dx
                    lookahead_y = y1 + t * seg_dy
    
                    # Transform to robot's local frame
                    dx = lookahead_x - robot_x
                    dy = lookahead_y - robot_y
                    local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
                    local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
    
                    # Make sure it's in front of the robot
                    if local_x > 0:
                        self.visited = i
                        return local_x, local_y
    
        # If no point found
        self.get_logger().info('Cannot find interpolated lookahead point')
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
