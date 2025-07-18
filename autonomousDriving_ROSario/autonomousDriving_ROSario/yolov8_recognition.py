# Nodo identificador con YOLO | Final-Term Challenge
# Equipo ROSario

# Importaciones necesarias
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import os
from datetime import datetime

# Cusstom Messages
from std_msgs.msg import Header 
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference


class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Cambiar dependiendo donde se ejecute
        self.model = YOLO(os.path.expanduser('~/rosario_ws/src/autonomousDriving_ROSario/models/yolov8n_rosario.v7.1.pt')).to('cuda')
        #self.model = YOLO(os.path.expanduser('~/rosario_ws/src/autonomousDriving_ROSario/models/yolov8n_rosario.v6.1.pt'))
        
        # Inicializacion de variables
        self.img = None
        self.bridge = CvBridge()

        self.video_writer_initialized = False
        self.out = None
        self.valid_img = False

        # Creacion de subscriptores y publicadores
        self.sub = self.create_subscription(Image, '/jetson_frame', self.camera_callback, 10)
        self.yolo_pub = self.create_publisher(Yolov8Inference, '/Yolov8_inference', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/inference_result', 10)
        
        # Frecuencia de muestreo
        self.node_hz = 10.0
        timer_period = 1.0/self.node_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        # Catch de Camara
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.get_logger().info('Failed to get an image')
            
    def timer_callback(self):
        # Logs de Debuggeo
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return
        
        # Procesado de Image
        frame_rotate = cv2.rotate(self.img, cv2.ROTATE_180)
        results = self.model(frame_rotate)

        # Inicializar el mensaje nuevo en cada ciclo
        self.yolo_msg = Yolov8Inference()
        self.yolo_msg.header = Header()
        self.yolo_msg.header.stamp = self.get_clock().now().to_msg()
        self.yolo_msg.header.frame_id = 'inference'

        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                conf = box.conf.item()
                inference_result.class_name = self.model.names[int(c)]
                inference_result.top = int(b[1])     # y1
                inference_result.left = int(b[0])    # x1
                inference_result.bottom = int(b[3])  # y2
                inference_result.right = int(b[2])   # x2
                inference_result.accuracy = float(conf)
                self.yolo_msg.yolov8_inference.append(inference_result)

        # Dibujar resultado
        frame = results[0].plot()

        if not self.video_writer_initialized:
            height, width, _ = frame.shape
            timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
            filename = f'{timestamp}_yolov8Inference.mp4'
            self.out = cv2.VideoWriter(filename,
                                    cv2.VideoWriter_fourcc(*'mp4v'),
                                    self.node_hz, (width, height))
            self.video_writer_initialized = True
            self.get_logger().info(f'VideoWriter inicializado con resolución: {width}x{height}')
        
        if self.out:
            self.out.write(frame)

        # Publicar imagen y resultados
        #self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        self.yolo_pub.publish(self.yolo_msg)

def main(args=None):
    rclpy.init(args=args)
    y_i = YoloInference()
    rclpy.spin(y_i)
    y_i.destroy_node()
    rclpy.shutdown()
    y_i.out.release()

if __name__ == '__main__':
    main()