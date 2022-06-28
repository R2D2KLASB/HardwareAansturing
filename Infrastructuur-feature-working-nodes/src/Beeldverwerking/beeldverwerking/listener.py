## @package beeldverwerking.listener
# Beeldverwerking listener
#
# An ROS2 (string) Listener node that will print the receveid string.
#
# Setup
# =====
# - ROS2 Listener Node



import rclpy
from .listener_node.listenerNode import Listener

def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()