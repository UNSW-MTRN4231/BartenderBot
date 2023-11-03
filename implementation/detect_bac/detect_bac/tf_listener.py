#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped

class ListenTransformNode(Node):

    def __init__(self):
        super().__init__('listen_tf_node')
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.create_timer(1.0, self.timer_callback)  # 1 second timer callback

    def timer_callback(self):
        try:
            trans = self.tfBuffer.lookup_transform('world', 'bottle_green', rclpy.time.Time())
            self.get_logger().info('Translation: x: %.2f, y: %.2f, z: %.2f' % (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z))
            self.get_logger().info('Rotation: x: %.2f, y: %.2f, z: %.2f, w: %.2f' % (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('TF2 Exception: Could not transform from bottle_green to base_link')

def main():
    rclpy.init()
    node = ListenTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
