# Nodo identificador de colores | Half-Term Challenge
# Equipo ROSario

import rclpy
import cv2
import numpy as np
import signal

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from datetime import datetime
from rcl_interfaces.msg import SetParametersResult

class lineDetector(Node):
    def __init__(self):
        super().__init__('lineDetector_node')

        self.declare_parameter('cut_por', 0.85)
        self.declare_parameter('mid_por', 0.6)
        self.declare_parameter('tLower_canny', 50)
        self.declare_parameter('tUpper_canny', 200)
        self.declare_parameter('blur_kernel', 3)
        self.declare_parameter('erode_kernel', 7)
        self.declare_parameter('dilate_kernel', 5)
        self.declare_parameter('iter_erode', 4)
        self.declare_parameter('iter_dilate', 2)
        self.declare_parameter('params_ready', False)

        self.cut_por = self.get_parameter('cut_por').value
        self.mid_por = self.get_parameter('mid_por').value
        self.tLower_canny = self.get_parameter('tLower_canny').value
        self.tUpper_canny = self.get_parameter('tUpper_canny').value
        self.blur_kernel = self.get_parameter('blur_kernel').value
        self.erode_kernel = self.get_parameter('erode_kernel').value
        self.dilate_kernel = self.get_parameter('dilate_kernel').value
        self.iter_erode = self.get_parameter('iter_erode').value
        self.iter_dilate = self.get_parameter('iter_dilate').value
        self.params_ready = self.get_parameter('params_ready').value
        self.img = None
        self.bridge = CvBridge()
        
        self.out = None
        
        self.video_writer_initialized = False

        self.target_width, self.target_height = 640, 480

        self.collage_size = (320, 240)
        
        self.line_error_msg = Float32()
        self.line_error_pub = self.create_publisher(Float32, 'line_error', 10)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        self.frecuencia_loop = 15.0
        self.controller_timer = self.create_timer(1.0 / self.frecuencia_loop, self.main_loop)

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
            elif param.name == "erode_kernel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.erode_kernel = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
            elif param.name == "dilate_kernel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.dilate_kernel = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
            elif param.name == "iter_erode":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.iter_erode = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
            elif param.name == "iter_dilate":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.iter_dilate = param.value  # Update internal variable
                    #self.get_logger().info(f"kd updated to {self.kd}")
            elif param.name == "mid_por":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("No puede ser negativo")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.mid_por = param.value  # Update internal variable
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

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')
    
    def calcular_error_ponderado(self, midpoint_centroids, frame_width):
        centro_x = frame_width // 2
        errores_normalizados = []

        # Calcular errores normalizados
        for centroide in midpoint_centroids:
            if centroide:
                error = centro_x - centroide[0] #centroide[0] - centro_x
                error_normalizado = error / (frame_width)  # valor entre -1 y 1
                errores_normalizados.append(error_normalizado)
            else:
                errores_normalizados.append(None)

        # Filtrar centroides válidos junto con sus errores
        centroides_validos = [(i, e) for i, e in enumerate(errores_normalizados) if e is not None]

        if not centroides_validos:
            return 0.0  # No hay datos válidos

        # Ordenar por distancia al centro (valor absoluto del error)
        centroides_ordenados = sorted(centroides_validos, key=lambda x: abs(x[1]), reverse=True)

        # Pesos: más lejano (0.5), medio (0.3), más cercano (0.2)
        pesos = [0.33, 0.33, 0.33]
        total_error = 0.0

        for idx, (i, error) in enumerate(centroides_ordenados):
            peso = pesos[idx] if idx < len(pesos) else 0
            total_error += peso * error

        return total_error
    
    def main_loop(self):

        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return

        if self.params_ready:
            centroids = [None] * 6  # 2 centroides por sección vertical

            flip_img = cv2.flip(self.img, 0)
            flip_img = cv2.flip(flip_img, 1)

            # --- Redimensionar imagen ---
            height, width = flip_img.shape[:2]
            if (width, height) != (self.target_width, self.target_height):
                resized_img = cv2.resize(flip_img, (self.target_width, self.target_height))
            else:
                resized_img = flip_img.copy()
            
            # --- Region de interés ---

            roi = resized_img[int(self.target_height * self.cut_por):, :]
            roi_height, roi_width = roi.shape[:2]
            
            # División del ancho del ROI en 15%, 70%, 15%
            w1 = int(roi_width * ((1.0-self.mid_por)/2))
            w2 = int(roi_width * self.mid_por)
            column_areas = [(0, w1), (w1, w1 + w2), (w1 + w2, roi_width)]
            roi = roi[:, w1:w1 + w2]

            output = roi.copy()

            # División de ROI en 3 secciones verticales iguales (por alto)
            third = roi_height // 3
            areas = [(0, third), (third, 2 * third), (2 * third, roi_height)]

            for i, (start_y, end_y) in enumerate(areas):
                sub_roi = roi[start_y:end_y, :]

                # Procesamiento por sub ROI
                gray = cv2.cvtColor(sub_roi, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
                binary_inv = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY_INV, 199, 5)

                # Operaciones morfológicas
                kernel = np.ones((self.erode_kernel, self.erode_kernel), np.uint8)
                morph_ero = cv2.erode(binary_inv, kernel, iterations=self.iter_erode)
                kernel = np.ones((self.dilate_kernel, self.dilate_kernel), np.uint8)
                morph_dil = cv2.dilate(morph_ero, kernel, iterations=self.iter_dilate)

                # Detección de bordes
                canny_edges = cv2.Canny(morph_dil, self.tLower_canny, self.tUpper_canny)

                # Búsqueda de contornos
                contours, _ = cv2.findContours(canny_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                local_centroids = []
                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx = x + w // 2
                    cy = y + h // 2 + start_y  # Ajuste al ROI global
                    local_centroids.append((cx, cy))
                    cv2.circle(output, (cx, cy), 7, (0, 0, 255), -1)

                # Guardar hasta 2 centroides por sección (rellenar con None si faltan)
                for j in range(2):
                    idx = i * 2 + j
                    if j < len(local_centroids):
                        centroids[idx] = local_centroids[j]

                cv2.drawContours(output[start_y:end_y], contours, -1, (0, 255, 0), 2)

            

            # Calcular punto medio de cada par (por sección)
            midpoint_centroids = []
            for i in [0,2,4]:
                c1, c2 = centroids[i], centroids[i+1]
                if c1 and c2:
                    mx = (c1[0] + c2[0]) // 2
                    my = (c1[1] + c2[1]) // 2
                    midpoint_centroids.append((mx, my))
                    cv2.circle(output, (mx, my), 5, (255, 0, 0), -1)
                else:
                    midpoint_centroids.append(None)
                    
            # Conteo de centroides en columnas (solo para los 6 originales)
            '''conteo_columnas = [0, 0, 0]
            for c in centroids:
                if c:
                    cx, _ = c
                    if cx < column_areas[0][1]:
                        conteo_columnas[0] += 1
                    elif cx < column_areas[1][1]:
                        conteo_columnas[1] += 1
                    else:
                        conteo_columnas[2] += 1'''

            '''# Dibujar líneas de separación de columnas
            for x in [column_areas[0][1], column_areas[1][1]]:
                cv2.line(output, (x, 0), (x, roi_height), (255, 255, 255), 2)  # línea blanca'''


            '''# Mostrar conteo por columnas
            for i, count in enumerate(conteo_columnas):
                x_text = column_areas[i][0] + 10
                cv2.putText(output, f"C{i+1}:{count}", (x_text, roi_height-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)'''

            error = self.calcular_error_ponderado(centroids, roi.shape[1])
            

            self.line_error_msg.data = float(error)
            self.line_error_pub.publish(self.line_error_msg)

            '''cv2.putText(output, f"Error: {error:.2f}", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)'''

            # --- COLLAGE de todo el proceso ---Add commentMore actions
            # Convertir grises a BGR para visualización conjunta
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            #blurred_bgr = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
            binary_inv_bgr = cv2.cvtColor(binary_inv, cv2.COLOR_GRAY2BGR)
            morphEro_bgr = cv2.cvtColor(morph_ero, cv2.COLOR_GRAY2BGR)
            morphDIL_bgr = cv2.cvtColor(morph_dil, cv2.COLOR_GRAY2BGR)
            canny_bgr = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2BGR)

            # Redimensionar todas las imágenesAdd commentMore actions
            img1 = cv2.resize(resized_img, self.collage_size)
            img2 = cv2.resize(roi, self.collage_size)
            img3 = cv2.resize(gray_bgr, self.collage_size)
            #img4 = cv2.resize(blurred_bgr, self.collage_size)
            img4 = cv2.resize(binary_inv_bgr, self.collage_size)
            img5 = cv2.resize(morphEro_bgr, self.collage_size)
            img6 = cv2.resize(morphDIL_bgr, self.collage_size)
            img7 = cv2.resize(canny_bgr, self.collage_size)
            img8 = cv2.resize(output, self.collage_size)

            # Concatenar en dos filas de cuatro imágenes
            row1 = cv2.hconcat([img1, img2, img3, img4])
            row2 = cv2.hconcat([img5, img6, img7, img8])
            collage = cv2.vconcat([row1, row2])

            # --- Creación de video de salida
            if not self.video_writer_initialized:
                height, width, _ = output.shape
                # Generar nombre dinámico: día_mes_año_hora_minuto_segundo
                timestamp = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
                filename = f'{timestamp}_lineDetector.mp4'

                self.out = cv2.VideoWriter(filename,
                                        cv2.VideoWriter_fourcc(*'mp4v'),
                                        self.frecuencia_loop, (4*self.collage_size[0], 2*self.collage_size[1]))

                self.video_writer_initialized = True
                self.get_logger().info("Video para el collage")

            # --- Escritura del video de salida ---
            if self.out:
                self.out.write(collage)

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        if self.out is not None:
            self.out.release()
            cv2.destroyAllWindows()
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