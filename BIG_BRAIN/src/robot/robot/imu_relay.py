import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Imu

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Point32, '/imu_msg', self.imu_callback, qos)
        self.publisher = self.create_publisher(Imu, '/imu', qos)

    def imu_callback(self, msg):
        imu_msg = Imu()
        
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = float(msg.x) * 9.81
        imu_msg.linear_acceleration.y = float(msg.y) * 9.81
        imu_msg.linear_acceleration.z = 9.81 
        
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = float(msg.z)*3.141592653/180

        # 3. Covariances (CRITICAL for the EKF)
        # You must provide realistic variance values (standard deviation squared) 
        # for the fields you are actually using. Do not leave them as 0.0.
        imu_msg.linear_acceleration_covariance[0] = 0.01 # Variance for Ax
        imu_msg.linear_acceleration_covariance[4] = 0.01 # Variance for Ay
        imu_msg.angular_velocity_covariance[8] = 0.01    # Variance for Gz (Vyaw)
        
        # Mark orientation as unknown since you don't calculate Roll/Pitch/Yaw on the STM32
        imu_msg.orientation_covariance[0] = -1.0 

        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()