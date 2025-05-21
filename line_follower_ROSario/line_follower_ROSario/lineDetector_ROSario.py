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
from rcl_interfaces.msg import SetParametersResult

class lineDetector(Node):
    def __init__(self):
        super().__init__('lineDetector_node')

        self.declare_parameter('cut_por', 0.525)
        self.declare_parameter('blur_kernel', 3)
        self.declare_parameter('morfo_kernel', 3)
        self.declare_parameter('params_ready', False)

        self.cut_por = self.get_parameter('cut_por').value
        self.blur_kernel = self.get_parameter('blur_kernel').value
        self.morfo_kernel = self.get_parameter('morfo_kernel').value
        self.params_ready = self.get_parameter('params_ready').value
        self.img = None
        self.bridge = CvBridge()
        
        self.out = None  # Se inicializa luego
        #self.out_raw = None
        
        self.video_writer_initialized = False
        #self.video_raw_writer_initialized = False

        self.target_width, self.target_height = 640, 480

        self.collage_size = (320, 240)

        self.color_id = 0

        self.line_error_pub = self.create_publisher(Int32, 'line_error', 10)
        self.line_error_msg = Int32()

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        frecuencia_controlador = 5.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Line Detector initialized!')

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "cut_por":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.cut_por = param.value  # Update internal variable
                    #self.get_logger().info(f"cut updated to {self.kp}")
            elif param.name == "blur_kernel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.blur_kernel = param.value  # Update internal variable
                    #self.get_logger().info(f"ki updated to {self.ki}")
            elif param.name == "morfo_kernel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.morfo_kernel = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
            elif param.name == "params_ready":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.params_ready = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
        return SetParametersResult(successful=True)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            '''if not self.video_raw_writer_initialized:
                height, width, _ = self.img.shape
                # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
                timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
                filename = f'{timestamp}_raw.mp4'

                self.out_raw = cv.VideoWriter(filename,
                                        cv.VideoWriter_fourcc(*'mp4v'),
                                        5, (width, height))

                self.video_raw_writer_initialized = True'''

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def calcular_error(self, centroids, frame_width):
        center_x = frame_width // 2
        
        if len(centroids) <= 2:
            # Solo una línea → curva
            cx = centroids[0][0]
            error = center_x - cx
            curva = True

        elif len(centroids) >= 3:
            # Recta o transición: promedio de centroides visibles
            avg_cx = int(np.mean([c[0] for c in centroids]))
            error = center_x - avg_cx
            curva = False

        else:
            # No hay líneas → sin control (puedes detener o mantener el último valor)
            error = 0
            curva = None  # No se puede decidir

        return error, curva


    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        if self.params_ready:
            centroids = []

            flip_img = cv.flip(self.img, 0)
            flip_img = cv.flip(flip_img, 1)

            # --- Redimensionar imagen ---
            height, width = flip_img.shape[:2]
            if (width, height) != (self.target_width, self.target_height):
                resized_img = cv.resize(flip_img, (self.target_width, self.target_height))
            else:
                resized_img = flip_img.copy()
            
            # --- Region de interés ---

            roi = resized_img[int(self.target_height * self.cut_por):, :]
            #roi_height, roi_width = roi.shape[:2]

            # --- Pre-procesado de la imagen ---
            gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
            blurred = cv.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
            _, binary_inv = cv.threshold(blurred, 100, 255, cv.THRESH_BINARY_INV)

            # --- Operaciones morfologicas ---
            kernel = np.ones((self.morfo_kernel, self.morfo_kernel), np.uint8)
            morph = cv.erode(binary_inv, kernel, iterations=3)
            morph = cv.dilate(morph, kernel, iterations=3)

            # --- Detección de bordes Canny ---
            canny_edges = cv.Canny(morph, 50, 200)

            # --- Busqueda de contornos ---
            contours, _ = cv.findContours(canny_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            output = roi.copy() # Imagen de salida

            # --- Impresión de contornos y su contador ---
            cv.drawContours(output, contours, -1, (0,255,0), 2)
            cv.putText(output, f"No. contornos: {len(contours)}", (50, 25), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)

            # --- Centroides ---
            for cnt in contours:
                x, y, w, h = cv.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                centroids.append((cx, cy))
                cv.circle(output, (cx, cy), 7, (0, 0, 255), -1)
            
            error, curva = self.calcular_error(centroids, roi.shape[1])
            self.line_error_msg.data = int(error)
            self.line_error_pub.publish(self.line_error_msg)
            cv.putText(output, f"Error: {error}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)
            cv.putText(output, f"Curve: {curva}", (50, 75), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)
            cv.putText(output, f"No. centroides: {len(centroids)}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)
            
            # --- COLLAGE de todo el proceso ---
            # Convertir grises a BGR para visualización conjunta
            gray_bgr = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)
            blurred_bgr = cv.cvtColor(blurred, cv.COLOR_GRAY2BGR)
            binary_inv_bgr = cv.cvtColor(binary_inv, cv.COLOR_GRAY2BGR)
            morph_bgr = cv.cvtColor(morph, cv.COLOR_GRAY2BGR)
            canny_bgr = cv.cvtColor(canny_edges, cv.COLOR_GRAY2BGR)

            # Redimensionar todas las imágenes
            img1 = cv.resize(resized_img, self.collage_size)
            img2 = cv.resize(roi, self.collage_size)
            img3 = cv.resize(gray_bgr, self.collage_size)
            img4 = cv.resize(blurred_bgr, self.collage_size)
            img5 = cv.resize(binary_inv_bgr, self.collage_size)
            img6 = cv.resize(morph_bgr, self.collage_size)
            img7 = cv.resize(canny_bgr, self.collage_size)
            img8 = cv.resize(output, self.collage_size)

            # Concatenar en dos filas de cuatro imágenes
            row1 = cv.hconcat([img1, img2, img3, img4])
            row2 = cv.hconcat([img5, img6, img7, img8])
            collage = cv.vconcat([row1, row2])

            # --- Creación de video de salida
            if not self.video_writer_initialized:
                height, width, _ = self.img.shape
                # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
                timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
                filename = f'{timestamp}_lineoutput.mp4'

                self.out = cv.VideoWriter(filename,
                                        cv.VideoWriter_fourcc(*'mp4v'),
                                        5, (4*self.collage_size[0], 2*self.collage_size[1]))

                self.video_writer_initialized = True
            
            # --- Escritura del video de salida ---
            if self.out:
                self.out.write(collage)
            
            '''if self.out_raw:
                self.out_raw.write(flip_img)'''

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        if self.out is not None:
            self.out.release()
            #self.out_raw.release()
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