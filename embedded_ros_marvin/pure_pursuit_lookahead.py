import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from infra_interfaces.action import FollowPath

import math
import time
import numpy as np
from scipy.interpolate import splprep, splev

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lookahead')
        self.get_logger().info('Pure Pursuit Node started.')

        # Parameters
        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.6
        self.base_lookahead = 0.1
        self.k_speed = 0.55
        self.visited = 0
        self.speed_factor = 1

        # State
        self.path = []
        self.raw_path = []
        self.pose = None
        self.reached_goal = False
        self.current_speed = 0.0

        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/joy_cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/debug_path', 10)
        self.raw_path_pub = self.create_publisher(Path, '/debug_raw_path', 10)

        self.create_timer(1.0, self.publish_path, callback_group=self.cb_group)
        self.create_timer(1.0, self.publish_raw_path, callback_group=self.cb_group)

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
        raw_path = [(p.x, p.y) for p in goal_handle.request.path]
        self.raw_path = raw_path
        self.path = self.smooth_path_spline(raw_path)
        self.reached_goal = False
        self.visited = 0

        while not self.reached_goal and rclpy.ok():
            time.sleep(0.05)

        self.path = []
        goal_handle.succeed()
        return FollowPath.Result()
    
  
    def smooth_path(self, path, window_size=5):
        if len(path) < window_size:
            return path

        # Separate into x and y arrays
        x = np.array([p[0] for p in path])
        y = np.array([p[1] for p in path])

        # Create a uniform kernel
        kernel = np.ones(2 * window_size + 1)
        kernel = kernel / kernel.sum()

        # Pad the data at the edges using 'edge' padding (repeat edge values)
        x_padded = np.pad(x, (window_size,), mode='edge')
        y_padded = np.pad(y, (window_size,), mode='edge')

        # Convolve to get smoothed values
        x_smooth = np.convolve(x_padded, kernel, mode='valid')
        y_smooth = np.convolve(y_padded, kernel, mode='valid')

        return list(zip(x_smooth, y_smooth))
    
    def smooth_path_spline(self, path, smoothing=0.1):
        if len(path) < 3:
            return path

        x = [p[0] for p in path]
        y = [p[1] for p in path]

        
        # Fit spline with no periodicity, and smoothing factor `s`
        tck, _ = splprep([x, y], s=smoothing, per=0)

        # Interpolate with 3 times the number of points
        num_points = 3 * len(path)
        u_fine = np.linspace(0, 1, num_points)
        x_smooth, y_smooth = splev(u_fine, tck)

        return list(zip(x_smooth, y_smooth))

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose = (pos.x, pos.y, self.get_yaw_from_quaternion(ori))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_lookahead_point(self):
        if self.pose is None or not self.path:
            return None

        x, y, yaw = self.pose
        goal_x, goal_y = self.path[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        self.lookahead_distance = max(0.1, min(0.4, self.base_lookahead + self.k_speed * self.current_speed))
        r = self.lookahead_distance

        if goal_dist < r:
            self.get_logger().info('REACHED GOAL')
            self.reached_goal = True
            return None


        # Try to find interpolated segment intersection
        # Allows one index backwards due to adaptive lookahead
        for i in range(max(0, self.visited - 1), len(self.path) - 1):
            gx1, gy1 = self.path[i]
            gx2, gy2 = self.path[i + 1]

            dx1, dy1 = gx1 - x, gy1 - y
            dx2, dy2 = gx2 - x, gy2 - y

            lx1 = math.cos(-yaw) * dx1 - math.sin(-yaw) * dy1
            ly1 = math.sin(-yaw) * dx1 + math.cos(-yaw) * dy1
            lx2 = math.cos(-yaw) * dx2 - math.sin(-yaw) * dy2
            ly2 = math.sin(-yaw) * dx2 + math.cos(-yaw) * dy2

            d1 = math.hypot(lx1, ly1)
            d2 = math.hypot(lx2, ly2)

            # Skip segment if it's completely behind
            if lx1 < -0.05 and lx2 < -0.05:
                self.visited = i + 1
                continue

            # If segment crosses the lookahead radius, do linear interpolation
            if d1 < r <= d2 and lx2 > 0.0:
                ratio = (r - d1) / (d2 - d1)
                ratio = max(0.0, min(1.0, ratio))  # clamp for safety
                lx = lx1 + ratio * (lx2 - lx1)
                ly = ly1 + ratio * (ly2 - ly1)
                self.visited = i
                return lx, ly

        # Fallback: no intersection with lookahead circle, pick the first forward point outside lookahead distance
        for j in range(self.visited, len(self.path)):
            gx, gy = self.path[j]
            dx, dy = gx - x, gy - y
            lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            d = math.hypot(lx, ly)
            if lx > -0.1 and d >= r:
                self.visited = j
                self.get_logger().warn(f'Fallback: chasing ahead point at index {j}')
                return lx, ly

        # Nothing usable found
        self.get_logger().warn('No valid lookahead point found – stopping')
        return None
    
    
    def control_loop(self):
        local_point = self.find_lookahead_point()
        if local_point is None:
            self.cmd_pub.publish(Twist())
            return
    
        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)

        # Scale up linear
        raw_linear = self.lookahead_distance * 1.5
    
        # More linear velocity on straights
        '''
        if abs(curvature) < 0.2:
            raw_linear *= 1.5
        '''
        raw_angular = raw_linear * curvature
    
        # Scale both linear and angular if angular exceeds limit
        # This keeps the proportions of pure pursuit
        if abs(raw_angular) > self.max_angular_speed:
            scale = self.max_angular_speed / abs(raw_angular)
            linear = raw_linear * scale
            angular = raw_angular * scale
        else:
            linear = raw_linear
            angular = raw_angular
    
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
    
    #original control loop
    '''
    def control_loop(self):
        local_point = self.find_lookahead_point()
        if local_point is None:
            self.cmd_pub.publish(Twist())
            return

        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)
        dist = math.hypot(local_x, local_y)

        linear = self.speed_factor * min(self.max_linear_speed, dist)
        angular = self.speed_factor * max(-self.max_angular_speed, min(self.max_angular_speed, linear * curvature))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
    '''
    
    def publish_path(self):
        if not self.path:
            return
        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"
        for x, y in self.path:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_raw_path(self):
        if not self.raw_path:
            return
        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"
        for x, y in self.raw_path:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.raw_path_pub.publish(path_msg)


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
