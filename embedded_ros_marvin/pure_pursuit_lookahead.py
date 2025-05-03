class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lookahead')
        self.get_logger().info('Pure Pursuit Node started.')

        # Parameters
        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.4
        self.base_lookahead = 0.25
        self.k_speed = 0.5  # scales with speed
        self.k_curve = 0.3  # reduces with curvature
        self.goal_tolerance = 0.3
        self.visited = 0

        # State
        self.path = []
        self.pose = None
        self.reached_goal = False
        self.current_speed = 0.0

        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/joy_cmd_vel', 10)
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
        return FollowPath.Result()

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

    def estimate_path_curvature(self, window=5):
        if len(self.path) < window:
            return 0.0
        total_curvature = 0.0
        count = 0
        for i in range(max(self.visited, 1), min(len(self.path)-1, self.visited + window)):
            x0, y0 = self.path[i - 1]
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]

            dx1, dy1 = x1 - x0, y1 - y0
            dx2, dy2 = x2 - x1, y2 - y1
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            dtheta = abs(angle2 - angle1)
            total_curvature += dtheta
            count += 1
        return total_curvature / count if count else 0.0

    def find_lookahead_point(self):
        if self.pose is None or not self.path:
            self.get_logger().info('Odom or Path not available')
            return None

        x, y, yaw = self.pose
        goal_x, goal_y = self.path[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        if goal_dist < self.goal_tolerance:
            self.get_logger().info('REACHED GOAL')
            self.reached_goal = True
            return None

        curvature = self.estimate_path_curvature()
        adaptive_lookahead = self.base_lookahead + self.k_speed * self.current_speed - self.k_curve * curvature
        adaptive_lookahead = max(0.1, min(1.5, adaptive_lookahead))  # clamp to avoid instability
        self.lookahead_distance = adaptive_lookahead
        self.get_logger().info(f'Adaptive lookahead: {self.lookahead_distance:.2f} m (speed={self.current_speed:.2f}, curve={curvature:.2f})')

        for i in range(self.visited, len(self.path)):
            gx, gy = self.path[i]
            dx = gx - x
            dy = gy - y
            local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            dist = math.hypot(local_x, local_y)
            if local_x > 0.0 and dist >= self.lookahead_distance:
                self.visited = i
                return local_x, local_y

        self.get_logger().info('Cannot find lookahead point')
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
