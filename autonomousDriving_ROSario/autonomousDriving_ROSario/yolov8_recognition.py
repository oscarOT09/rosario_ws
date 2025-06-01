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

class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO(os.path.expanduser('~/rosario_ws/src/autonomousDriving_ROSario/models/yolov8n_rosario.v1.pt')) #self.model = YOLO('~/rosario_ws/src/autonomousDriving_ROSario/models/oscarObjects_best.pt') #YOLO('~/ros2_cadi_ws/src/yolov8_ros2/yolov8_ros2/yolov8x.pt')
        self.yolo_msg = Yolov8Inference()
        self.img = np.ndarray((720, 1280, 3))
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.yolo_pub = self.create_publisher(Yolov8Inference, '/Yolov8_inference', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/inference_result', 10)
        
        node_hz =2.0
        timer_period = 1.0/node_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        try:
            #self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = cv2.rotate(img, cv2.ROTATE_180)
            self.valid_img = True
            self.get_logger().info('Imagen recibida')
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        results = self.model(self.img)
        self.yolo_msg.header.frame_id = 'inference'
        self.yolo_msg.header.stamp = self.get_clock().now().to_msg()

        for r in results:
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
                self.yolo_msg.yolov8_inference.append(self.inference_result)
        
        frame = results[0].plot()
        #self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        frame_uint8 = frame.astype(np.uint8)
        self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame_uint8, encoding='bgr8'))
        self.get_logger().info('Publicado Image con inferencia')
        self.yolo_pub.publish(self.yolo_msg)
        self.get_logger().info('Publicado Yolov8Inference')
        self.yolo_msg.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    y_i = YoloInference()
    rclpy.spin(y_i)
    y_i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
