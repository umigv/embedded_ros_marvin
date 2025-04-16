import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from infra_interfaces.action import FollowPath  # Import the action
import math
import threading
from scipy.spatial.transform import Rotation



class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lookahead')
        self.get_logger().info('Pure Pursuit Node started.')
        # Parameters
        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.4
        self.lookahead_distance = 0.3

        # State
        self.pose = None
        self.pose_lock = threading.Lock()
        self.action_complete_cv = threading.Condition()
        self.control_timer = None   
        self.reset_action_state()

        self.cb_group = ReentrantCallbackGroup()
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_follow_path,
            callback_group=self.cb_group
        )

    def reset_action_state(self):
        self.path = []
        self.reached_goal = False
        if self.control_timer:
            self.control_timer.cancel()
            self.control_timer = None

    def execute_follow_path(self, goal_handle):
        self.path = [(p.x, p.y) for p in goal_handle.request.path]
        self.get_logger().info(f'Received a new path from action client: {self.path}')
        self.control_timer = self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)
        with self.action_complete_cv:
            while not self.reached_goal:
                self.action_complete_cv.wait()
        self.reset_action_state()
        goal_handle.succeed()
        result = FollowPath.Result()
        result.success = True
        self.get_logger().info('FollowPath action completed')
        return result

    def control_loop(self):
        local_point = self.find_lookahead_point()

        if local_point is None:
            self.cmd_pub.publish(Twist())  # Stop
            if  self.reached_goal:
                self.get_logger().info('Path completed.')
                with self.action_complete_cv:
                    self.action_complete_cv.notify_all()
            else:
                self.get_logger().info('Waiting for vaild lookahead point.')
            return

        self.get_logger().info(f'Lookahead point: {local_point}')
        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x**2 + local_y**2)
        dist = math.hypot(local_x, local_y)

        linear = min(self.max_linear_speed, dist)
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, linear * curvature))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd) 

    def odom_callback(self, msg):
        # self.get_logger().info('Received odometry data')
        with self.pose_lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = self.get_yaw_from_quaternion(ori)
            # yaw *= -1
            self.pose = (pos.x, pos.y, yaw)
            # self.get_logger().info(f'{self.pose}')

    def get_yaw_from_quaternion(self, q):
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return yaw

    def find_lookahead_point(self):
        with self.pose_lock:
            if not self.pose: 
                return
            x, y, yaw = self.pose

        for gx, gy in self.path:
            dx = gx - x
            dy = gy - y

            # Transform to robot's frame
            local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            dist = math.hypot(local_x, local_y)

            # Prevents driving backwards or directly to the side
            if dist >= self.lookahead_distance:
                return local_x, local_y

        self.reached_goal = True
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()