import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration


class UMBmarkSequence(Node):
    def __init__(self):
        super().__init__('umbmark_node')

        # Enable sim time for Gazebo
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        self.publisher_ = self.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)


        # UMBmark square parameters
        side_length = 4.0      # meters
        linear_speed = 0.5     # m/s
        angular_speed = 0.5    # rad/s
        direction = -1           # 1 for CCW, -1 for CW

        import math
        straight_time = side_length / linear_speed
        turn_time = (math.pi / 2) / angular_speed

        # Sequence: (Duration, Linear X, Angular Z)
        self.sequence = []
        for _ in range(4):
            self.sequence.append((straight_time, linear_speed, 0.0))
            self.sequence.append((turn_time, 0.0, direction * angular_speed))

        self.current_step = 0
        self.step_end_time = None
        self.clock_synced = False
        self.started = False

        self.timer = self.create_timer(0.01, self.timer_callback)

        # Subscribe to ground truth odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def timer_callback(self):
        now = self.get_clock().now()

        # Wait for Gazebo clock to be valid
        if now.nanoseconds == 0:
            return

        if not self.clock_synced:
            self.clock_synced = True
            self.start_time = now
            self.get_logger().info('Clock synced, starting in 0.5s...')
            return

        # Startup delay
        if now < self.start_time + Duration(seconds=0.5):
            return

        if self.current_step >= len(self.sequence):
            self.publisher_.publish(Twist())
            self.get_logger().info('Sequence complete')
            self.get_logger().info(f'Final position: x={self.current_x:.3f}, y={self.current_y:.3f}')
            rclpy.shutdown()
            return

        if self.step_end_time is None:
            duration, _, _ = self.sequence[self.current_step]
            self.step_end_time = now + Duration(seconds=duration)
            self.get_logger().info(f'Step {self.current_step + 1}: {duration:.2f}s')

        if now < self.step_end_time:
            _, lin_x, ang_z = self.sequence[self.current_step]
            msg = Twist()
            msg.linear.x = lin_x
            msg.angular.z = ang_z
            self.publisher_.publish(msg)
        else:
            self.publisher_.publish(Twist())
            self.current_step += 1
            self.step_end_time = None


def main(args=None):
    rclpy.init(args=args)
    node = UMBmarkSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
