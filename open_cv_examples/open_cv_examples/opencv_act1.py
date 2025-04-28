import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

class ColorDetectionNode(Node):

    def __init__(self):
        super().__init__('color_detection_node')

        # Initialize variables
        self.img = None
        self.bridge = CvBridge()

        # Define HSV range for blue color detection
        self.blue_lower = np.array([100, 150, 50], np.uint8)
        self.blue_upper = np.array([130, 255, 255], np.uint8)

        self.green_lower = np.array([60, 100, 100], np.uint8)
        self.green_upper = np.array([90, 255, 255], np.uint8)

        self.red1_lower = np.array([0, 60, 50], np.uint8)
        self.red1_upper = np.array([10, 255, 255], np.uint8)

        self.red2_lower = np.array([130, 50, 50], np.uint8)
        self.red2_upper = np.array([178, 255, 255], np.uint8)

        # ROS2 Subscriber for raw camera images
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',#'/image_raw',
            self.camera_callback,
            10
        )

        # ROS2 Publisher for processed images
        self.image_pub = self.create_publisher(
            Image,
            '/image_processing/image',
            10
        )

        # Timer setup for regular processing
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Color Detection Node has started!')

    def camera_callback(self, msg):
        """Callback to receive image from the camera and convert to CV2 format."""
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')

    def timer_callback(self):
        """Timer callback for processing the image."""

        # Check if image has been received
        if self.img is None:
            self.get_logger().info('Waiting for image data...')
            return
        flip_img = cv.flip(self.img, 0)
        flip_img = cv.flip(flip_img, 1)
        # Step 1: Reduce noise with Gaussian blur
        blurred_img = cv.GaussianBlur(flip_img, (9, 9), 2)

        # Step 2: Convert from BGR to HSV color space
        hsv_img = cv.cvtColor(blurred_img, cv.COLOR_BGR2HSV)

        # Step 3: Create binary masks for colors
        red_mask1  = cv.inRange(hsv_img, self.red1_lower, self.red1_upper)
        red_mask2  = cv.inRange(hsv_img, self.red2_lower, self.red2_upper)
        red_mask   = cv.bitwise_or(red_mask1,red_mask2)
        green_mask = cv.inRange(hsv_img, self.blue_lower, self.blue_upper)
        blue_mask  = cv.inRange(hsv_img, self.blue_lower, self.blue_upper)

        temp_mask = cv.bitwise_or(hsv_img,red_mask,green_mask)
        color_mask = cv.bitwise_or(hsv_img,temp_mask,blue_mask)

        # Step 4: Apply mask to extract blue regions from original image
        extracted_img = cv.bitwise_and(flip_img, flip_img, mask=color_mask)

        # Step 5: Convert extracted blue regions to grayscale
        gray_color_img = cv.cvtColor(extracted_img, cv.COLOR_BGR2GRAY)

        # Step 6: Threshold grayscale image to binary image
        _, binary_blue_img = cv.threshold(gray_color_img, 5, 255, cv.THRESH_BINARY)

        # Step 7: Apply morphological operations to clean noise     (Apertura)
        kernel = np.ones((3, 3), np.uint8)
        cleaned_img = cv.erode(binary_blue_img, kernel, iterations=8)
        cleaned_img = cv.dilate(cleaned_img, kernel, iterations=8)

        # Publish the cleaned binary image
        processed_img_msg = self.bridge.cv2_to_imgmsg(cleaned_img, encoding='mono8')
        self.image_pub.publish(processed_img_msg)


def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()

    try:
        rclpy.spin(color_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        color_detection_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()