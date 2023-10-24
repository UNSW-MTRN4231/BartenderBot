#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from yolov8_msgs.msg import DetectionArray
from vision_ros_msgs.msg import BoundingBox, BoundingBoxes

class YoloOpencvProcessor(Node):

    def __init__(self):
        super().__init__('yolo_opencv_processor')

        callback_group = ReentrantCallbackGroup()

        self.yolo_subscription = self.create_subscription(
            DetectionArray, '/yolo/detections', self.yolo_listener_callback, 10, callback_group=callback_group)
        self.vision_subscription = self.create_subscription(
            BoundingBoxes, '/vision_opencv', self.vision_listener_callback, 10, callback_group=callback_group)

        self.bboxes_pub = self.create_publisher(BoundingBoxes, 'detect_bac', 10)

        self.latest_yolo_detections = None
        self.latest_vision_boxes = None

        self.timer = self.create_timer(0.1, self.process_data)

    def yolo_listener_callback(self, msg: DetectionArray):
        self.latest_yolo_detections = msg

    def vision_listener_callback(self, msg: BoundingBoxes):
        self.latest_vision_boxes = msg

    def process_data(self):
        if self.latest_yolo_detections is None or self.latest_vision_boxes is None:
            return

        self.get_logger().info('Processing data from both YOLO and OpenCV...')

        for vision_box in self.latest_vision_boxes.bounding_boxes:
            center_x = vision_box.x
            center_y = vision_box.y
            opencv_class = vision_box.color

            yolo_class_inside = None
            for yolo_detection in self.latest_yolo_detections.detections:
            
                if yolo_detection.class_name not in ['bottle', 'cup']:
                    continue
            
                yolo_center_x = yolo_detection.bbox.center.position.x
                yolo_center_y = yolo_detection.bbox.center.position.y
                yolo_width = yolo_detection.bbox.size.x
                yolo_height = yolo_detection.bbox.size.y

                left = yolo_center_x - (yolo_width / 2)
                right = yolo_center_x + (yolo_width / 2)
                top = yolo_center_y - (yolo_height / 2)
                bottom = yolo_center_y + (yolo_height / 2)

                if left <= center_x <= right and top <= center_y <= bottom:
                    yolo_class_inside = yolo_detection.class_name
                    break

            if yolo_class_inside:
                combined_class = f"{yolo_class_inside}_{opencv_class}"
                self.get_logger().info(f'OpenCV detection (center: {center_x}, {center_y}) is inside a YOLO {yolo_class_inside} bounding box. 
                     Combined class: {combined_class}')
                
                transform_matrix = np.array([[1, 0, 0],
                                            [0, 1, 0]])

                center_matrix = np.array([[center_x, center_y]])

                result = np.matmul(center_matrix, transform_matrix)
                
                transformed_x = result[0, 0]
                transformed_y = result[0, 1]
                
                bbox_msg = BoundingBox()
                bbox_msg.x = transformed_x
                bbox_msg.y = transformed_y
                bbox_msg.angle = 0.0  # Assuming no rotation
                bbox_msg.color = combined_class
            
                bboxes_msg = BoundingBoxes()
                bboxes_msg.bounding_boxes = [bbox_msg]
            
                self.bboxes_pub.publish(bboxes_msg)

            else:
                self.get_logger().info(f'OpenCV detection (center: {center_x}, {center_y}) is NOT inside any YOLO bounding box.')


def main(args=None):
    rclpy.init(args=args)
    processor = YoloOpencvProcessor()
    rclpy.spin(processor)

    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
