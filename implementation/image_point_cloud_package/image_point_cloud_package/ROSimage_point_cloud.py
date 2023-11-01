#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import pyrealsense2 as rs2
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO

# When launching the camera with ROS use the following:
# ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true
# The important thing here is the align_depth.enable:=true, which creates a topic which has the pointcloud points aligned with the RGB image

# GENERAL GUIDELINES AND RANGES:
# From experimenting, the depth camera can handle having an object directly in front of it at a min range of 10cm 

# HOWEVER, if the object is at the edges of the camera/pointcloud view, the minimum distance is about 15-20cm.
# Any closer and the wrong pixels get overlayed on the wrong pointcloud points.

class ImagePointCloudNode(Node):
    def __init__(self):
        super().__init__('image_point_cloud_node')

        # This just get the direct image from the camera
        self.subscription_image = self.create_subscription( Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # This gets the depth_frame aligned with the RGB image (pointcloud points that correspond with pixels of image)
        self.subscription_point_cloud = self.create_subscription( Image, '/camera/aligned_depth_to_color/image_raw', self.point_cloud_callback, 10)
        
        # This gets information about camrea (intrinsics)
        self.subscription_cam_info = self.create_subscription( CameraInfo, '/camera/depth/camera_info', self.imageDepthInfoCallback,10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        

        self.subscription_image  # Prevent unused variable warning
        self.subscription_point_cloud  # Prevent unused variable warning
        self.subscription_cam_info

        self.cv_bridge = CvBridge()
        self.x = 100
        self.y = 100
        self.DEBUG = True
        plt.ion()
            
        self.xPoints = []  # Initialize x as a list
        self.yPoints = []  # Initialize y as a list


        self.calculated_trajectory = False
        self.first_identify = True

        self.intrinsics = None


    
    # This gets the camrea parameters from ros topic (Needed to get x,y,z coordinates of pointcloud points)
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return


    # This gets bgr image from the image topic and finds where green in the image is
    def image_callback(self, msg):        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            
            # Perform green mask operation
            green_lower = np.array([0, 100, 0], dtype=np.uint8)
            green_upper = np.array([100, 255, 100], dtype=np.uint8)
            mask = cv2.inRange(cv_image, green_lower, green_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    self.x = int(M["m10"] / M["m00"])
                    self.y = int(M["m01"] / M["m00"])

                cv_image = cv2.circle(cv_image, (self.x,self.y), radius=3, color=(0, 0, 255), thickness=20)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

        
    # This gets depth_frame aligned with RGB image and finds the depth and coordinates of the green point
    def point_cloud_callback(self, msg):
        try:
            if self.x is not None and self.y is not None:
                # Process point cloud and find the global coordinates at x, y
                

                # Pixel (0,0) is the top left
                # WARNING:
                # Depth is not the Euclidean distance from the camera, it is the distance along the Z-axis (axis pointing out of camera)
                # This is why depth value and z coordinate value match
                # For Eucludean distance just use formula is easy as 
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (self.x, self.y, cv_image[self.y, self.x])

                depth = cv_image[self.y, self.x]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [int(self.x), int(self.y)], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], -result[1], result[2]) # Pos x is right side of image, pos y is bottom half of image, hence the negative sign 
                print(line)

                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = 'camera_link'
                transform_stamped.child_frame_id = 'target'
                transform_stamped.transform.translation.x = result[2]/1000
                transform_stamped.transform.translation.y = -result[0]/1000
                transform_stamped.transform.translation.z = -result[1]/1000

                self.tf_broadcaster.sendTransform(transform_stamped)
                

        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = ImagePointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


