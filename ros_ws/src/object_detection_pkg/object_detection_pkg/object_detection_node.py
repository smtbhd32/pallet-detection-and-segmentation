import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import supervision as sv
import sys

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.declare_parameter('weight_folder', '')

        # Subscribe to the camera's RGB image
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to the camera's depth image
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.depth_image = None  # Store the latest depth image

        # Get the weight folder parameter
        self.weight_folder = self.get_parameter('weight_folder').get_parameter_value().string_value

        # Define model file names
        detection_model_filename = 'detect_weight/best.pt'
        segmentation_model_filename = 'segment_weight/best.pt'

        # Construct the full paths to the model weights
        detection_model_path = os.path.join(self.weight_folder, detection_model_filename)
        segmentation_model_path = os.path.join(self.weight_folder, segmentation_model_filename)

        if not os.path.exists(detection_model_path) or not os.path.exists(segmentation_model_path):
            self.get_logger().error("Model file(s) not found!")
            return

        self.detection_model = YOLO(detection_model_path)  # Load YOLO detection model
        
        self.segmentation_model = YOLO(segmentation_model_path)  # Load YOLO segmentation model

    def image_callback(self, msg):
        # Convert ROS2 image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform object detection on the frame
        results_detection = self.simple_detection(frame)

        # Perform segmentation on the frame
        results_segmentation = self.simple_segmentation(frame)
        
        # Display results
        cv2.imshow("Detected Objects", results_detection)
        cv2.imshow("Segmentation", results_segmentation)
        cv2.imshow("Original Frame", frame)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        # Convert depth image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.get_logger().info("Depth image received!")

    def simple_segmentation(self, frame):
        # Perform segmentation using YOLO
        results = self.segmentation_model.predict(frame,conf=0.4)[0]  # Frame is already an OpenCV image (NumPy array)

        if not results or results.boxes is None:  # Check for empty results
            self.get_logger().warn("No detections made in the current frame.")
            return frame
    
        # Visualize the result (draw segmentation output)
        detections = sv.Detections.from_ultralytics(results)
        mask_annotator = sv.MaskAnnotator()
        annotated_image = frame.copy()
        mask_annotator.annotate(annotated_image, detections=detections)
        return annotated_image

        
    
    def simple_detection(self, frame):
        # Perform detection using YOLO
        results = self.detection_model.predict(frame,conf=0.4)[0] # Frame is already an OpenCV image (NumPy array)
        
        # Visualize the result (draw boxes, etc.)
        if not results or results.boxes is None:  # Check for empty results
            self.get_logger().warn("No detections made in the current frame.")
            return frame
    
        # Visualize the result (draw segmentation output)
        detections = sv.Detections.from_ultralytics(results)

        # Annotate boxes and labels
        box_annotator = sv.BoxAnnotator()
        label_annotator = sv.LabelAnnotator()
        annotated_image = frame.copy()
        annotated_image = box_annotator.annotate(annotated_image, detections=detections)
        annotated_image = label_annotator.annotate(annotated_image, detections=detections)
        return annotated_image


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl + C pressed. Shutting down gracefully.\n")
        if rclpy.ok():
            node.get_logger().info('Node terminated.')
    finally:
        # Ensure context is active before shutdown
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

        # Explicitly disable further logging to prevent warnings
        rclpy.logging._root_logger = None

        sys.exit(0)


if __name__ == '__main__':
    main()
