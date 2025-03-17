# Dynamic-ROS-2-Publisher-and-Subscriber-Node
# Dynamic-ROS-2-Publisher-and-Subscriber-Node

Overview

This ROS 2 node allows dynamic creation of publishers and subscribers for multiple topics. It also generates dummy data and publishes it periodically for testing.

Features

Dynamically creates publishers and subscribers for different topics.

Supports multiple message types (String, LaserScan, Pose).

Periodically publishes dummy data.

Logs received messages.

Installation

Make sure you have ROS 2 installed and sourced:

source /opt/ros/<your_ros_distro>/setup.bash

Clone the repository and navigate to the script:

git clone <repo_url>
cd <repo_directory>

Dependencies

Ensure you have the required ROS 2 Python packages installed:

pip install rclpy

How to Run

Execute the Python script:

python3 multi_pub_sub.py

Expected Output

The script logs publisher and subscriber creation along with sent and received messages:

[INFO] [dynamic_pub_sub_node]: Added Publisher on /string_topic
[INFO] [dynamic_pub_sub_node]: Added Subscriber on /string_topic
[INFO] [dynamic_pub_sub_node]: Published message on /string_topic
[INFO] [dynamic_pub_sub_node]: Received on /string_topic: Hello, ROS 2!

Code Explanation

1️⃣ Import Required Modules

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile

2️⃣ Create the DynamicPubSubNode Class

class DynamicPubSubNode(Node):
    def __init__(self):
        super().__init__('dynamic_pub_sub_node')
        self.custom_publishers = {}
        self.custom_subscribers = {}
        self.create_timer(1.0, self.publish_dummy_data)

Stores publishers & subscribers dynamically.

Calls publish_dummy_data() every 1 second.

3️⃣ Dynamically Add Publishers

def add_publisher(self, topic_name, msg_type, qos=10):
    self.custom_publishers[topic_name] = self.create_publisher(msg_type, topic_name, qos)

Registers a publisher for a given topic.

4️⃣ Dynamically Add Subscribers

def add_subscriber(self, topic_name, msg_type, qos=10):
    def callback(msg):
        self.get_logger().info(f'Received on {topic_name}: {self.extract_message(msg)}')
    self.custom_subscribers[topic_name] = self.create_subscription(msg_type, topic_name, callback, qos)

Listens to messages and extracts data.

5️⃣ Publishing Dummy Data

def publish_dummy_data(self):
    string_msg = String()
    string_msg.data = "Hello, ROS 2!"
    self.publish_message('/string_topic', string_msg)

Sends random test data.

6️⃣ Running the Node

def main():
    rclpy.init()
    node = DynamicPubSubNode()
    node.add_publisher('/string_topic', String)
    node.add_subscriber('/string_topic', String)
    rclpy.spin(node)
    rclpy.shutdown()

Initializes, adds publishers/subscribers, and runs the node.

Customization

Modify add_publisher() and add_subscriber() to handle new message types.

License

This project is licensed under the MIT License.

Contact

For questions or contributions, open an issue or reach out to the repository owner.
