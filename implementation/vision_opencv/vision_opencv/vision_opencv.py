#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt 
from vision_ros_msgs.msg import BoundingBox, BoundingBoxes

BOX_COLORS = {
    "red": {"color_lower": np.array([0, 127, 67]), "color_upper": np.array([10, 255, 255])},
    "blue": {"color_lower": np.array([100, 183, 136]), "color_upper": np.array([124, 255, 255])},
    #"orange": {"color_lower": np.array([0, 101, 146]), "color_upper": np.array([24, 255, 255])},
    "yellow": {"color_lower": np.array([31, 43, 144]), "color_upper": np.array([78, 255, 255])},
    "green": {"color_lower": np.array([66, 88, 76]), "color_upper": np.array([90, 255, 191])},
    #"purple": {"color_lower": np.array([125, 43, 46]), "color_upper": np.array([155, 255, 255])},
    "pink": {"color_lower": np.array([153, 58, 137]), "color_upper": np.array([166, 255, 255])}
    
}

class ImageConverter(Node):

    def __init__(self):
        super().__init__('image_converter')
       
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.callback, 10)
        self.position_pub = self.create_publisher(
            BoundingBoxes, "/vision_opencv", 10)
        self.bridge = CvBridge()

    def cv_show(self, name, img):
        cv2.imshow(name, img)
        cv2.waitKey(3)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        boundingBoxes = BoundingBoxes()
        frame_ = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_, cv2.COLOR_BGR2HSV)

        for color in BOX_COLORS.keys():
            mask = cv2.inRange(hsv, BOX_COLORS[color]["color_lower"], BOX_COLORS[color]["color_upper"])
            #plt.imshow(mask)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            #plt.show()
            mask = cv2.GaussianBlur(mask, (3, 3), 0)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            for cnt in cnts:

                area = cv2.contourArea(cnt)
                print(area)

                if area >500:

                    x, y, w, h = cv2.boundingRect(cnt)
                    rect = cv2.minAreaRect(cnt)
                    rotation = rect[2]
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    boundingBox = BoundingBox()
                    boundingBox.color = color
                    boundingBox.angle = rotation
                    boundingBox.x = np.float64(x + int(w/2.0))
                    boundingBox.y = np.float64(y + int(h/2.0))

                    boundingBoxes.bounding_boxes.append(boundingBox)
        self.position_pub.publish(boundingBoxes)

        self.cv_show("img", frame)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    print('start')

    try:
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        print('exception')
        image_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()