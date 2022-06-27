## @package beeldverwerking.publisher_node.publisherNode
# an ROS2 publisher node that sends an string.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

## ROS2 Publisher Node
class Publisher(Node):

    ## Constructor
    def __init__(self):
        super().__init__('publisher')
        ## The ROS2 Topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    ## Send string over ROS2
    def send(self, image):
        msg = String()
        msg.data = image
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)