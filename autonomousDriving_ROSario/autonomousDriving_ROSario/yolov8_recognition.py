import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import os

from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from datetime import datetime

class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO(os.path.expanduser('~/rosario_ws/src/autonomousDriving_ROSario/models/yolov8n_rosario.v5.2.pt')) #self.model = YOLO('~/rosario_ws/src/autonomousDriving_ROSario/models/oscarObjects_best.pt') #YOLO('~/ros2_cadi_ws/src/yolov8_ros2/yolov8_ros2/yolov8x.pt')
        #self.yolo_msg = Yolov8Inference()
        self.img = None
        self.bridge = CvBridge()

        self.video_writer_initialized = False
        self.out = None
        self.valid_img = False

        self.sub = self.create_subscription(Image, '/jetson_frame', self.camera_callback, 10)
        #self.yolo_pub = self.create_publisher(Yolov8Inference, '/Yolov8_inference', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/inference_result', 10)
        
        self.node_hz = 15.0
        timer_period = 1.0/self.node_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info('Imagen recibida')
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return
        
        frame_rotate = cv2.rotate(self.img, cv2.ROTATE_180)
        results = self.model(frame_rotate)
        #self.yolo_msg.header.frame_id = 'inference'
        #self.yolo_msg.header.stamp = self.get_clock().now().to_msg()

        '''for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolo_msg.yolov8_inference.append(self.inference_result)'''
        
        frame = results[0].plot()

        if not self.video_writer_initialized:
            height, width, _ = frame.shape

            # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
            timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
            filename = f'{timestamp}_yolov8Inference.mp4'

            self.out = cv2.VideoWriter(filename,
                                    cv2.VideoWriter_fourcc(*'mp4v'),
                                    self.node_hz, (width, height))

            self.video_writer_initialized = True
            self.get_logger().info('VideoWriter inicializado con resolución: {}x{}'.format(width, height))
        
        if self.out:
            self.out.write(frame)

        #self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        #frame_uint8 = frame.astype(np.uint8)
        self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        #self.get_logger().info('Publicado Image con inferencia')
        #self.yolo_pub.publish(self.yolo_msg)
        #self.get_logger().info('Publicado Yolov8Inference')
        #self.yolo_msg.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    y_i = YoloInference()
    rclpy.spin(y_i)
    y_i.destroy_node()
    rclpy.shutdown()
    y_i.out.release()

if __name__ == '__main__':
    main()
