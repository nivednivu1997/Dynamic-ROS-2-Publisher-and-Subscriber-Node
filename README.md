# Dynamic-ROS-2-Publisher-and-Subscriber-Node


## Overview

This ROS 2 node allows dynamic creation of publishers and subscribers for multiple topics. It also generates dummy data and publishes it periodically for testing.

## Features

- Dynamically creates publishers and subscribers for different topics.

- Supports multiple message types (String, LaserScan, Pose).

- Periodically publishes dummy data.

- Logs received messages.

## Installation

Make sure you have ROS 2 installed and sourced:
```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

Clone the repository and navigate to the script:
```bash
git clone https://github.com/nivednivu1997/Dynamic-ROS-2-Publisher-and-Subscriber-Node.git
```



## Dependencies

Ensure you have the required ROS 2 Python packages installed:
```bash
pip install rclpy
```
## How to Run

Execute the Python script:
```bash
python3 multi_pub_sub.py
```

## Customization

Modify add_publisher() and add_subscriber() to handle new message types.

