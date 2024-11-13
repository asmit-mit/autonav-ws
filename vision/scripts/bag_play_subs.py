import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

class MultiTopicSubscriber(Node):
    def __init__(self):
        super().__init__('multi_topic_subscriber')

        # Define list of topics and their message types
        topics = [
            ('/clock', Clock),
            ('/ouster/points', PointCloud2),
            ('/zed/zed_node/point_cloud/cloud_registered', PointCloud2),
            ('/zed/zed_node/depth/depth_registered', Image),
            ('/monocular_camera/image_raw', Image),
            ('/dlo/odom_node/odom', Odometry),
            ('/tf', TFMessage),
            ('/tf_static', TFMessage)
        ]

        # Create a subscription for each topic
        for topic_name, msg_type in topics:
            self.create_subscription(msg_type, topic_name, self.generic_callback, 10)

    # A generic callback function that handles all topics
    def generic_callback(self, msg):
        self.get_logger().info(f'Received message from topic: {msg._topic_name}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
