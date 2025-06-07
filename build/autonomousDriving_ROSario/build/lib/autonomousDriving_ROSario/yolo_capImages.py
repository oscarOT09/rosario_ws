import rclpy
import cv2 as cv
import signal
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
import os

class capImages(Node):
    def __init__(self):
        super().__init__('capImages_node')

        self.img = None
        self.bridge = CvBridge()
        self.take_photo = False
        self.photo_counter = 0
        self.yolo_class = ''
        self.yolo_class_folder = ''

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        self.declare_parameter('take_photo', False)
        self.declare_parameter('yolo_class', '')

        self.get_logger().info('Camera Recorder initialized!')

        self.timer = self.create_timer(0.1, self.main_loop)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        self.take_photo = self.get_parameter('take_photo').get_parameter_value().bool_value
        self.yolo_class = self.get_parameter('yolo_class').get_parameter_value().string_value

        if self.take_photo:
            self.take_photo_callback()
            self.set_parameters([Parameter('take_photo', Parameter.Type.BOOL, False)])

    def take_photo_callback(self):
        if not self.yolo_class:
            self.get_logger().error('No yolo_class specified')
            return

        self.yolo_class_folder = f'./yolo_images/{self.yolo_class}'
        if not os.path.exists(self.yolo_class_folder):
            os.makedirs(self.yolo_class_folder)

        existing_images = [f for f in os.listdir(self.yolo_class_folder) if f.startswith(self.yolo_class) and f.endswith('.jpg')]
        self.photo_counter = len(existing_images)

        filename = f'{self.yolo_class_folder}/{self.yolo_class}_{self.photo_counter}.jpg'
        flip_img = cv.flip(self.img, 0)
        flip_img = cv.flip(flip_img, 1)
        cv.imwrite(filename, flip_img)
        self.get_logger().info(f'Foto guardada: {filename}')
        self.photo_counter += 1

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = capImages()
    signal.signal(signal.SIGINT, node.stop_handler)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Nodo finalizado limpiamente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()