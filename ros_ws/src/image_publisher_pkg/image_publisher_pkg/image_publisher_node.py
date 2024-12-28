import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from time import sleep

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.declare_parameter('image_folder', '')
        self.declare_parameter('publish_rate', 1)

        # Get parameters
        self.image_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        # Publisher
        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)

        # Timer to publish images at a specific rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)

        # Bridge to convert OpenCV images to ROS image messages
        self.bridge = CvBridge()

        # Read image files from the specified folder
        self.image_files = [f for f in os.listdir(self.image_folder) if f.endswith(('.png', '.jpg'))]
        self.index = 0

        if not self.image_files:
            self.get_logger().warn(f'No images found in the folder: {self.image_folder}')
    
    def publish_image(self):
        if not self.image_files:
            self.get_logger().warn('No images found in the folder!')
            return

        image_path = os.path.join(self.image_folder, self.image_files[self.index])
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(ros_image)

        # Log the actual image being published
        self.get_logger().info(f"Publishing image: {self.image_files[self.index]}")

        # Update index to cycle through images
        self.index = (self.index + 1) % len(self.image_files)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
