## @package beeldverwerking.listener_node.listenerNode
# an simple ROS2 listener node that prints the receveid string.

from rclpy.node import Node
from std_msgs.msg import String

## Listener ROS2 Node
class Listener(Node):

    ##Constructor
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    ## Print receveid string.
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
