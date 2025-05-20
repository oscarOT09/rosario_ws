# Nodo identificador de colores | Half-Term Challenge
# Equipo ROSario

import rclpy
import cv2 as cv
import numpy as np
import signal

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from datetime import datetime


class lineDetector(Node):
    def __init__(self):
        super().__init__('lineDetector_node')

        self.img = None
        self.bridge = CvBridge()
        
        self.out = None  # Se inicializa luego
        self.out_raw = None
        
        self.video_writer_initialized = False
        self.video_raw_writer_initialized = False

        self.target_width, self.target_height = 640, 480

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

        self.get_logger().info('Line Detector initialized!')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.video_raw_writer_initialized:
                height, width, _ = self.img.shape
                # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
                timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
                filename = f'{timestamp}_raw.mp4'

                self.out_raw = cv.VideoWriter(filename,
                                        cv.VideoWriter_fourcc(*'mp4v'),
                                        5, (width, height))

                self.video_raw_writer_initialized = True

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        flip_img = cv.flip(self.img, 0)
        flip_img = cv.flip(flip_img, 1)

        # Resize image if needed
        height, width = flip_img.shape[:2]
        if (width, height) != (self.target_width, self.target_height):
            resized_img = cv.resize(flip_img, (self.target_width, self.target_height))
            #print("Image resized to 640x480.")
        else:
            resized_img = flip_img.copy()
            #print("Image already 640x480.")    

        # --- Step 1: Create trapezoidal mask ---

        #roi = resized_img[int(target_height*0.0):, :]
        roi = resized_img[int(self.target_height * 0.4):, :]

        roi_height, roi_width = roi.shape[:2]

        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)

        top_width = int(roi_width * 0.9)
        trapezoid = np.array([[
            ((roi_width - top_width) // 2, int(roi_height * 0.0)),  # top-left
            ((roi_width + top_width) // 2, int(roi_height * 0.0)),  # top-right
            (roi_width, roi_height),                                # bottom-right
            (0, roi_height)                                            # bottom-left
        ]], dtype=np.int32)

        cv.fillPoly(mask, trapezoid, 255)
        roi = cv.bitwise_and(roi, roi, mask=mask)

        # --- Step 2: Preprocess image ---
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        _, binary_inv = cv.threshold(blurred, 100, 255, cv.THRESH_BINARY_INV)

        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        morph = cv.erode(binary_inv, kernel, iterations=3)
        morph = cv.dilate(morph, kernel, iterations=3)

        # Canny edge detection
        canny_edges = cv.Canny(morph, 50, 150)

        side_crop_percent = 0.05
        crop_x = int(roi_width * side_crop_percent)

        # Create a mask that only keeps the center part
        side_mask = np.zeros_like(canny_edges)
        cv.rectangle(
            side_mask,
            (crop_x, 0),  # top-left corner
            (roi_width - crop_x, roi_height),  # bottom-right corner
            255,  # white region
            thickness=-1
        )

        # Apply side mask
        canny_edges = cv.bitwise_and(canny_edges, canny_edges, mask=side_mask)

        # Apply trapezoidal mask to Canny output
        all_edges_roi = cv.bitwise_and(canny_edges, canny_edges, mask=mask)

        # --- Step 3: Find and process contours ---
        contours, _ = cv.findContours(all_edges_roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        output = roi.copy()
        epsilon_factor = 0.1

        for cnt in contours:
            # Approximate polygon
            epsilon = epsilon_factor * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)

            # Bounding rectangle and center
            x, y, w, h = cv.boundingRect(approx)
            cv.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue box

            box_area = w * h

            # Filter by bounding box area
            if box_area < 1000 or box_area > 100000:
                continue

            cx = x + w // 2
            cy = y + h // 2
            cv.circle(output, (cx, cy), 4, (0, 0, 255), -1)               # Red center
            cv.putText(output, f"({cx},{cy})", (cx + 10, cy), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            # Draw the approximated polygon
            cv.polylines(output, [approx], isClosed=True, color=(0, 255, 0), thickness=2)

        # Convert to RGB for matplotlib
        output_rgb = cv.cvtColor(output, cv.COLOR_BGR2RGB)

        #cv.imshow('Output', output_rgb)
        if not self.video_writer_initialized:
            height, width, _ = self.img.shape
            # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
            timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
            filename = f'{timestamp}_lineoutput.mp4'

            self.out = cv.VideoWriter(filename,
                                    cv.VideoWriter_fourcc(*'mp4v'),
                                    5, (output_rgb.shape[1], output_rgb.shape[0]))

            self.video_writer_initialized = True
        
        if self.out:
            self.out.write(output)
        
        if self.out_raw:
            self.out_raw.write(flip_img)

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        if self.out is not None:
            self.out.release()
            self.out_raw.release()
            cv.destroyAllWindows()
            self.get_logger().info('VideoWriter liberado y ventanas cerradas.')
        raise SystemExit
    
def main(args=None):
    rclpy.init(args=args)
    node = lineDetector()
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