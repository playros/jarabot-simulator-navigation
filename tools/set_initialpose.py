import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    rclpy.init()
    node = Node('set_initialpose_once')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0

    cov = [0.0]*36
    cov[0] = 0.5
    cov[7] = 0.5
    cov[35] = 0.2
    msg.pose.covariance = cov

    for _ in range(10):
        pub.publish(msg)

    node.get_logger().info("✅ Published /initialpose")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
