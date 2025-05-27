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
from rosario_path.msg import RosarioPath

class lineDetector(Node):
    def __init__(self):
        super().__init__('lineDetector_node')

        self.declare_parameter('cut_por', 0.55)
        self.declare_parameter('blur_kernel', 3)
        self.declare_parameter('morfo_kernel', 5)
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

        self.line_error_pub = self.create_publisher(RosarioPath, 'line_error', 10)
        self.line_error_msg = RosarioPath()

        #Curva
        self.smoothed_error = 0
        self.smoothing_factor = 0.3  # 0 (no change), 1 (instant change)
        self.MAX_ERROR = 200

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        frecuencia_loop = 5.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_loop, self.main_loop)

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

    def ransac_polyfit(self, x, y, degree = 3, max_trials = 100, threshold = 10):
        best_inliers = None
        best_coef = None
        best_inlier_num = 0

        n_points = len(x)
        if n_points < degree + 1:
            return None, None

        for i in range(max_trials):
           # Randomly select minimal subset to fit polynomial
           indices = np.random.choice(n_points, degree + 1, replace = False)
           x_subset = x[indices]
           y_subset = y[indices]

           try:
                coef = np.polyfit(y_subset, x_subset, degree)
                p = np.poly1d(coef)
                residuals = np.abs(x - p(y))
                inliers = residuals < threshold
                inlier_count = np.sum(inliers)
                if inlier_count > best_inlier_num:
                    best_inlier_num = inlier_count
                    best_inliers = inliers
                    best_coef = coef
           except np.RankWarning:
                # Skip badly conditioned fits
                continue
           except Exception:
                continue
           
        if best_coef is None:
            return None, None
        try:
            coef = np.polyfit(y[best_inliers], x[best_inliers], degree)
            best_poly = np.poly1d(coef)
            return best_poly, best_inliers
        except Exception:
            return np.poly1d(best_coef), best_inliers

           
    def calcular_error(self, centroids, frame_width, all_points, debug_img=None):
        center_x = frame_width // 2
        closest_cx = None

        if len(centroids) <= 2 and len(all_points) >= 6:
            all_points = np.array(all_points)
            x = all_points[:, 0]
            y = all_points[:, 1]

            if len(np.unique(y)) < 4:
                # Puntos no suficientes para el cubic fit
                if debug_img is not None:
                    cv.putText(debug_img, 'Datos insuficientes para ajuste cúbico.', (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                error = 0
                curva = False
                closest_cx = None

            else:
                # Use RANSAC polynomial fitting for robust estimation
                poly, inliers = self.ransac_polyfit(x, y, degree = 3, max_trials = 200, threshold = 15)
                if poly is None:
                    error = 0
                    curva = False
                    closest_cx = 0

                else:
                    
                    y_target = max(y)
                    x_on_bottom = int(poly(y_target))

                    if np.isfinite(x_on_bottom):
                        raw_error = center_x - x_on_bottom
                        raw_error = max(min(raw_error, self.MAX_ERROR), -self.MAX_ERROR)
                        
                        coef = poly.coefficients
                        coef_cubico = coef[0]
                        if abs(coef_cubico) > 0.0004:
                            curva = True
                        else:
                            curva = False
                        
                        closest_cx = None

                        # Error suave con media móvil exponencial para estabilizar la salida
                        self.smoothed_error = (self.smoothing_factor * raw_error + (1 - self.smoothing_factor)* self.smoothed_error)
                        error = int(self.smoothed_error)

                        # Dibujar la curva en la imagen de salida (si se proporciona)
                        if debug_img is not None:
                            y_range = np.linspace(min(y), y_target, 50)
                            for i in range(len(y_range)-1):
                                pt1 = (int(poly(y_range[i])), int(y_range[i]))
                                pt2 = (int(poly(y_range[i+1])), int(y_range[i+1]))
                                if np.isfinite(pt1[0]) and np.isfinite(pt2[0]):
                                    cv.line(debug_img, pt1, pt2, (255, 0, 255), 2)
                            cv.circle(debug_img, (x_on_bottom, int(y_target)), 5, (0, 0, 255), -1)
                            # Visualize inliers and outliers
                            for (px, py), inlier in zip(zip(x, y), inliers):
                                color = (0, 255, 0) if inlier else (0, 0, 255)
                                cv.circle(debug_img, (int(px), int(py)), 3, color, -1)
                    else:
                        self.get_logger().warn("Valor cúbico no finito.")
                        error = 0
                        curva = False
                        closest_cx = None

        elif len(centroids) >= 3:
            # Error recto: centroide más cercano al centro de imagen
            min_dist = float('inf')
            for c in centroids:
                cx = c[0]
                dist = abs(cx - center_x)
                if dist < min_dist:
                    min_dist = dist
                    closest_cx = cx

            raw_error = center_x - closest_cx
            
            # Smooth error
            self.smoothed_error = (self.smoothing_factor * raw_error +
                                   (1 - self.smoothing_factor) * self.smoothed_error)
            error = int(self.smoothed_error)
            curva = False

        else:
            error = 0
            curva = False
            closest_cx = None
            # Reset smoothing when no data
            self.smoothed_error = 0

        return (error/frame_width), curva, closest_cx




    def main_loop(self):

        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        if self.params_ready:
            centroids = []
            all_points = []

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
            #_, binary_inv = cv.threshold(blurred, 100, 255, cv.THRESH_BINARY_INV)
            
            binary_inv = cv.adaptiveThreshold(blurred, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv.THRESH_BINARY_INV, 199, 5)
            
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

                for point in cnt:
                    all_points.append(point[0])
            
            if len(centroids) > 0 and len(all_points) > 2:
                error, curva, closest_cx = self.calcular_error(centroids, roi.shape[1], all_points, debug_img=output)
            else:
                error = 0
                curva = False
                closest_cx = None



            self.line_error_msg.error = float(error)
            self.line_error_msg.curva = curva
            self.line_error_pub.publish(self.line_error_msg)
            
            cv.putText(output, f"Error: {error}", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)
            cv.putText(output, f"Curve: {curva}", (50, 75), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)
            cv.putText(output, f"No. centroides: {len(centroids)}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 1)

            
            
            if closest_cx is not None:
                roi_height = roi.shape[0]
                cv.line(output, (closest_cx, 0), (closest_cx, roi_height), (255, 0, 255), 2)
                cv.putText(output, "Centro seg.", (closest_cx - 30, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                
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