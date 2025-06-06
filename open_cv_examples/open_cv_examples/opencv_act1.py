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
        self.yellow_lower = np.array([25, 20, 165], np.uint8)
        self.yellow_upper = np.array([35, 200, 255], np.uint8)

        self.green_lower = np.array([60, 100, 100], np.uint8)
        self.green_upper = np.array([80, 255, 255], np.uint8)

        self.red1_lower = np.array([0, 100, 0], np.uint8)
        self.red1_upper = np.array([5, 255, 255], np.uint8)

        self.red2_lower = np.array([175, 100, 0], np.uint8)
        self.red2_upper = np.array([180, 255, 255], np.uint8)

        self.white_lower = np.array([0, 10, 200], np.uint8)
        self.white_upper = np.array([80, 255, 250], np.uint8)     

        self.color_id = 0


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
        self.timer = self.create_timer(1.0/16.0, self.timer_callback)

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

        #flip_img = cv.flip(self.img, 0)
        #flip_img = cv.flip(self.img, 1)

        alto = self.img.shape[0]

        corte = int(alto*0.7)
        flip_img = self.img[:corte,:]

        # Step 1: Reduce noise with Gaussian blur
        blurred_img = cv.GaussianBlur(flip_img, (9, 9), 2)

        # Step 2: Convert from BGR to HSV color space
        hsv_img = cv.cvtColor(blurred_img, cv.COLOR_BGR2HSV)

        # Step 3: Create binary masks for colors
        red_mask1   = cv.inRange(hsv_img, self.red1_lower, self.red1_upper)
        red_mask2   = cv.inRange(hsv_img, self.red2_lower, self.red2_upper)
        mask_red    = cv.bitwise_or(red_mask1,red_mask2)
        mask_green  = cv.inRange(hsv_img, self.green_lower, self.green_upper)
        mask_yellow = cv.inRange(hsv_img,self.yellow_lower, self.yellow_upper)
        mask_white  = cv.inRange(hsv_img,self.white_lower, self.white_upper)

        kernel =  cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))

        mask_trafic = cv.morphologyEx(mask_white, cv.MORPH_OPEN, kernel, iterations=3)
        
        # Detectar círculos
        mask_blur = cv.GaussianBlur(mask_trafic, (9, 9), 2)
        circles = cv.HoughCircles(mask_blur, cv.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=50, param2=30, minRadius=5, maxRadius=100)
        
        # Crear máscara circular expandida
        circle_mask = np.zeros(mask_trafic.shape, dtype=np.uint8)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for (x, y, r) in circles[0, :]:
                cv.circle(circle_mask, (x, y), r + r//10, 255, -1)       

        # Resultados
            resultado = cv.bitwise_and(flip_img,flip_img, mask=circle_mask)
            mask_red = cv.bitwise_and(mask_red, circle_mask)
            mask_yellow = cv.bitwise_and(mask_yellow, circle_mask)
            mask_green = cv.bitwise_and(mask_green, circle_mask)

        # Conteo de píxeles
        red_count = cv.countNonZero(mask_red)
        yellow_count = cv.countNonZero(mask_yellow)
        green_count = cv.countNonZero(mask_green)
        
        # Determinar el color encendido
        if max(red_count, yellow_count, green_count) > 150:  # Umbral mínimo
            if red_count == max(red_count, yellow_count, green_count):
                self.color_id = 1
            elif yellow_count == max(red_count, yellow_count, green_count):
                self.color_id = 2
            elif green_count == max(red_count, yellow_count, green_count):
                self.color_id = 3
        else:
            self.color_id = 0
                
        color_mask = cv.bitwise_or(mask_red, mask_green)
        color_mask = cv.bitwise_or(color_mask,mask_yellow)


        # Paso 8 : Aplicar la mascara a la imagen original para ver los colores
        resultado = cv.bitwise_and(flip_img, flip_img, mask=color_mask)
        resultado_rgb = cv.cvtColor(resultado, cv.COLOR_BGR2RGB)

        # Publish the cleaned binary image
        processed_img_msg = self.bridge.cv2_to_imgmsg(resultado_rgb, encoding='rgb8')
        self.get_logger().info(f"Color detectado: {self.color_id}")
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