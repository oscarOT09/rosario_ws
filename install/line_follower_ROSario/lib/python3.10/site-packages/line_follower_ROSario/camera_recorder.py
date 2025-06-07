"""
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

class cameraRecorder(Node):
    def __init__(self):
        super().__init__('cameraRecorder_node')

        self.img = None
        self.bridge = CvBridge()
        self.take_photo = False
        self.photo_counter = 0
        self.category = ''
        self.category_folder = ''

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        self.declare_parameter('take_photo', False)
        self.declare_parameter('category', '')

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
        self.category = self.get_parameter('category').get_parameter_value().string_value

        if self.take_photo:
            self.take_photo_callback()
            self.set_parameters([Parameter('take_photo', Parameter.Type.BOOL, False)])

    def take_photo_callback(self):
        if not self.category:
            self.get_logger().error('No category specified')
            return

        self.category_folder = f'./imagenes/{self.category}'
        if not os.path.exists(self.category_folder):
            os.makedirs(self.category_folder)

        timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
        filename = f'{self.category_folder}/{timestamp}_{self.photo_counter}.jpg'
        cv.imwrite(filename, self.img)
        self.get_logger().info(f'Foto guardada: {filename}')
        self.photo_counter += 1

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = cameraRecorder()
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
"""
# Nodo identificador de colores | Half-Term Challenge
# Equipo ROSario

import rclpy
import cv2 as cv
import signal

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime


class cameraRecorder(Node):
    def __init__(self):
        super().__init__('cameraRecorder_node')

        self.img = None
        self.bridge = CvBridge()
        self.video_writer_initialized = False
        self.out = None  # Se inicializa luego

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        frecuencia_controlador = 5.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Camera Recorder initialized!')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Inicializa el video solo una vez (después de recibir el primer frame)
            if not self.video_writer_initialized:
                height, width, _ = self.img.shape
                '''self.out = cv.VideoWriter('output.mp4',
                                            cv.VideoWriter_fourcc(*'mp4v'),
                                            5, (width, height))'''
                # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
                timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
                filename = f'{timestamp}_raw.mp4'

                self.out = cv.VideoWriter(filename,
                                        cv.VideoWriter_fourcc(*'mp4v'),
                                        5, (width, height))

                self.video_writer_initialized = True
                self.get_logger().info('VideoWriter inicializado con resolución: {}x{}'.format(width, height))

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return
        
        if self.out:
            flip_img = cv.flip(self.img, 0)
            flip_img = cv.flip(flip_img, 1)
            self.out.write(flip_img)

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
    node = cameraRecorder()
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