#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from vision_ros_msgs.msg import BoundingBoxes
from grasp_helper.srv import CamToReal
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup


class TfBroadcast(Node):
    def __init__(self):
        super().__init__('tf_broadcast')

        self.node = rclpy.create_node('tf_broadcast1')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.callback_group = ReentrantCallbackGroup()
        self.dist_client = self.create_client(CamToReal, '/cam_to_real', callback_group=self.callback_group)
        self.subscription = self.create_subscription(
                    BoundingBoxes,
                    'detect_bac',
                    self.msg_callback,
                    10,
                    callback_group=self.callback_group
                )

    def msg_callback(self, msg):
        for bbox in msg.bounding_boxes:
            print(1)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_link'
            transform.child_frame_id = bbox.color

            pixel_x = bbox.x
            pixel_y = bbox.y

            dist_req = CamToReal.Request()
            dist_req.pixel_x = pixel_x
            dist_req.pixel_y = pixel_y
            print(dist_req)

            future = self.dist_client.call_async(dist_req)
            # self.dist_client.fu
            rclpy.spin_until_future_complete(self.node, future)
            dist_resp = future.result()
            print(dist_resp)
            # rclpy.spin_until_future_complete(self, future)

            if dist_resp is not None and dist_resp.obj_z!= 0 :
                transform.transform.translation.x = dist_resp.obj_x
                transform.transform.translation.y = dist_resp.obj_y
                transform.transform.translation.z = dist_resp.obj_z
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    tf_broadcast = TfBroadcast()
   
    try:
        rclpy.spin(tf_broadcast)
    except KeyboardInterrupt:
        pass

    tf_broadcast.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
