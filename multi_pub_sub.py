import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile

class DynamicPubSubNode(Node):
    def __init__(self):
        super().__init__('dynamic_pub_sub_node')
        self.custom_publishers = {}  # Renamed to avoid conflicts
        self.custom_subscribers = {}  # Renamed to avoid conflicts

        # Timer to publish dummy data every second
        self.create_timer(1.0, self.publish_dummy_data)

    def add_publisher(self, topic_name, msg_type, qos=10):
        self.custom_publishers[topic_name] = self.create_publisher(msg_type, topic_name, qos)
        self.get_logger().info(f'Added Publisher on {topic_name}')

    def add_subscriber(self, topic_name, msg_type, qos=10):
        def callback(msg):
            extracted_data = self.extract_message(msg)
            self.get_logger().info(f'Received on {topic_name}: {extracted_data}')

        self.custom_subscribers[topic_name] = self.create_subscription(msg_type, topic_name, callback, qos)
        self.get_logger().info(f'Added Subscriber on {topic_name}')

    def publish_message(self, topic_name, msg):
        if topic_name in self.custom_publishers:
            self.custom_publishers[topic_name].publish(msg)
            self.get_logger().info(f'Published message on {topic_name}')
        else:
            self.get_logger().warn(f'Publisher for {topic_name} not found!')

    def extract_message(self, msg):
        if isinstance(msg, String):
            return msg.data
        elif isinstance(msg, LaserScan):
            return f'LaserScan: min_range={msg.range_min}, max_range={msg.range_max}'
        elif isinstance(msg, Pose):
            return f'Pose: x={msg.position.x}, y={msg.position.y}'
        return "[Unsupported Type]"

    def publish_dummy_data(self):
        """Publishes dummy messages for testing"""
        # String message
        string_msg = String()
        string_msg.data = "Hello, ROS 2!"
        self.publish_message('/string_topic', string_msg)

        # LaserScan message (random data)
        laser_msg = LaserScan()
        laser_msg.range_min = 0.2
        laser_msg.range_max = 10.0
        laser_msg.ranges = [random.uniform(0.2, 10.0) for _ in range(10)]
        self.publish_message('/laser_scan_topic', laser_msg)

        # Pose message (random position)
        pose_msg = Pose()
        pose_msg.position.x = random.uniform(-5.0, 5.0)
        pose_msg.position.y = random.uniform(-5.0, 5.0)
        pose_msg.position.z = 0.0
        self.publish_message('/pose_topic', pose_msg)

def main():
    rclpy.init()
    node = DynamicPubSubNode()

    # Adding publishers
    node.add_publisher('/string_topic', String)
    node.add_publisher('/laser_scan_topic', LaserScan)
    node.add_publisher('/pose_topic', Pose)

    # Adding subscribers
    node.add_subscriber('/string_topic', String)
    node.add_subscriber('/laser_scan_topic', LaserScan)
    node.add_subscriber('/pose_topic', Pose)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
