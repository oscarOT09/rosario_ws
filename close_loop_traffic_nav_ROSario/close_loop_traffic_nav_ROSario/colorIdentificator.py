'''# Importaciones necesarias
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Int32

# Definición de la clase
class colorIdentificator(Node):
    def __init__(self):
        super().__init__('color_identificator')

        # Initialize variables
        self.img = None
        self.bridge = CvBridge()

        # Define HSV range for blue color detection
        self.yellow_lower = np.array([25, 15, 195], np.uint8)
        self.yellow_upper = np.array([35, 180, 255], np.uint8)

        self.green_lower = np.array([60, 100, 100], np.uint8)
        self.green_upper = np.array([80, 255, 255], np.uint8)

        self.red1_lower = np.array([0, 100, 0], np.uint8)
        self.red1_upper = np.array([5, 255, 255], np.uint8)

        self.red2_lower = np.array([175, 100, 0], np.uint8)
        self.red2_upper = np.array([180, 255, 255], np.uint8)

        self.white_lower = np.array([0, 0, 233], np.uint8)
        self.white_upper = np.array([180, 180, 255], np.uint8)     

        self.N = 25
        self.color_id = 0

        self.out = cv.VideoWriter('output.mp4',
                               cv.VideoWriter_fourcc(*'mp4v'),
                               10, (640, 480))
        
        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.color_pub = self.create_publisher(Int32, 'color_id', 10)
        self.color_msg = Int32()

        # ROS2 Publisher for processed images
        self.image_pub = self.create_publisher(
            Image,
            '/image_processing/image',
            10
        )

        # ROS2 Subscriber for raw camera images
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',#'/image_raw',
            self.camera_callback,
            10
        )

        # Muestreo
        frecuencia_controlador = 5.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Color Identificator initialized!')
    
    def camera_callback(self, msg):
        """Callback to receive image from the camera and convert to CV2 format."""
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.out.write(self.img)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')

    def main_loop(self):
    
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
        circles = cv.HoughCircles(mask_blur, cv.HOUGH_GRADIENT, dp=1.5, minDist=30,
                                param1=50, param2=133, minRadius=15, maxRadius=100)

        # Crear máscara circular expandida
        circle_mask = np.zeros(mask_trafic.shape, dtype=np.uint8)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for (x, y, r) in circles[0, :]:
                cv.circle(circle_mask, (x, y), r + self.N, 255, -1)       

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
        if max(red_count, yellow_count, green_count) > 100:  # Umbral mínimo
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
        self.image_pub.publish(processed_img_msg)

        self.color_msg.data = self.color_id
        self.color_pub.publish(self.color_msg)
        self.get_logger().info(f"Color detectado: {self.color_id}")


# Main
def main(args=None):
    rclpy.init(args=args)
    node = colorIdentificator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.out.release()
        rclpy.shutdown()

# Execute
if __name__ == '__main__':
    main()'''

# -------------------------------------------------------------------------------------

import rclpy
import cv2 as cv
import numpy as np
import signal

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32

class colorIdentificator(Node):
    def __init__(self):
        super().__init__('color_identificator')

        self.img = None
        self.bridge = CvBridge()
        self.video_writer_initialized = False
        self.out = None  # Se inicializa luego

        # Rango HSV para diferentes colores
        self.yellow_lower = np.array([25, 15, 195], np.uint8)
        self.yellow_upper = np.array([35, 180, 255], np.uint8)
        self.green_lower  = np.array([60, 100, 100], np.uint8)
        self.green_upper  = np.array([80, 255, 255], np.uint8)
        self.red1_lower   = np.array([0, 100, 0], np.uint8)
        self.red1_upper   = np.array([5, 255, 255], np.uint8)
        self.red2_lower   = np.array([175, 100, 0], np.uint8)
        self.red2_upper   = np.array([180, 255, 255], np.uint8)
        self.white_lower  = np.array([0, 0, 233], np.uint8)
        self.white_upper  = np.array([180, 180, 255], np.uint8)

        self.N = 25
        self.color_id = 0

        self.color_pub = self.create_publisher(Int32, 'color_id', 10)
        self.color_msg = Int32()

        self.image_pub = self.create_publisher(Image, '/image_processing/image', 10)

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        frecuencia_controlador = 5.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Color Identificator initialized!')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Inicializa el video solo una vez (después de recibir el primer frame)
            if not self.video_writer_initialized:
                height, width, _ = self.img.shape
                self.out = cv.VideoWriter('output.mp4',
                                          cv.VideoWriter_fourcc(*'mp4v'),
                                          5, (width, height))
                self.video_writer_initialized = True
                self.get_logger().info('VideoWriter inicializado con resolución: {}x{}'.format(width, height))

            # Guarda el frame
            if self.out is not None:
                self.out.write(self.img)

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        # Aquí vendría tu lógica de procesamiento de color...

    '''def destroy_node(self):
        # Libera el VideoWriter si fue inicializado
        if self.out is not None:
            self.out.release()
            cv.destroyAllWindows()
            self.get_logger().info('VideoWriter liberado y ventanas cerradas.')
        super().destroy_node()'''

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        if self.out is not None:
            self.out.release()
            cv.destroyAllWindows()
            self.get_logger().info('VideoWriter liberado y ventanas cerradas.')
        raise SystemExit
    
def main(args=None):
    rclpy.init(args=args)
    node = colorIdentificator()
    signal.signal(signal.SIGINT, node.stop_handler)

    '''try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()'''
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Nodo finalizado limpiamente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# -------------------------------------------------------------------------------------

'''# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
#import transforms3d

#from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
#from rclpy.qos import qos_profile_sensor_data
#from rosario_path.msg import RosarioPath

# Definición de la clase
class colorIdentificator(Node):
    def __init__(self):
        super().__init__('color_identificator')
        
        # Muestreo
        frecuencia_controlador = 16.0

        self.state_start_time = self.get_clock().now()

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.color_pub = self.create_publisher(Int32, 'color_id', 10)
        self.color_msg = Int32()

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Color Identificator initialized!')
    
    def main_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        if elapsed >= 0 and elapsed <= 2:
            self.color_msg.data = 3
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')
        elif elapsed >= 2 and elapsed <= 5:
            self.color_msg.data = 2
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')
        elif elapsed >= 5 and elapsed <= 7:
            self.color_msg.data = 1
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')
        elif elapsed >= 7 and elapsed <= 10:
            self.color_msg.data = 0
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')


# Main
def main(args=None):
    rclpy.init(args=args)
    node = colorIdentificator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Execute
if __name__ == '__main__':
    main()'''