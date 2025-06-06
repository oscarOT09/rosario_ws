import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime

class framesPublisher(Node):
    def __init__(self):
        super().__init__('frames_publisher_node')

        self.img = None
        self.bridge = CvBridge()

        self.valid_img = False

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.frame_pub = self.create_publisher(Image, '/jetson_frame', 10)
        
        self.node_hz = 15.0
        timer_period = 1.0/self.node_hz
        self.timer = self.create_timer(timer_period, self.frames2pc_pub)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.get_logger().info('Failed to get an image')

    def frames2pc_pub(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return
        
        self.frame_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = framesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
