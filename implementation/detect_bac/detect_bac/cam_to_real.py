#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from grasp_helper.srv import CamToReal
import cv2
import numpy as np

class ImageConverter(Node):

    def __init__(self):
        super().__init__('detect_obj')

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = np.zeros((720, 1280), dtype=np.uint16)
        self.camera_info = None

        self.image_sub_depth = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.image_depth_callback, 10)
        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, "/camera/aligned_depth_to_color/camera_info", self.camera_info_callback, 10)
        self.cam_to_real = self.create_service(
            CamToReal, "/cam_to_real", self.cam_to_real_callback)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")


    def cam_to_real_callback(self, request, response):
        obj_x = request.pixel_x
        obj_y = request.pixel_y
        if 0 <= int(obj_x) < self.depth_image.shape[1] and 0 <= int(obj_y) < self.depth_image.shape[0]:
            real_z = self.depth_image[int(obj_y), int(obj_x)] / 1000.0
            if real_z != 0:
                real_x = (obj_x - self.camera_info.k[2]) / self.camera_info.k[0] * real_z
                real_y = (obj_y - self.camera_info.k[5]) / self.camera_info.k[4] * real_z
                response.obj_x = np.float64(real_x)
                response.obj_y = np.float64(real_y)
                response.obj_z = np.float64(real_z)
                response.result = True
            else:
                response.obj_x = np.float64(0)
                response.obj_y = np.float64(0)
                response.obj_z = np.float64(0)
                response.result = False

        else:
            self.get_logger().warning(f'Invalid pixel coordinates: x={obj_x}, y={obj_y}')

        return response

def main():
    rclpy.init()
    ic = ImageConverter()
    rclpy.spin(ic)
    ic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
