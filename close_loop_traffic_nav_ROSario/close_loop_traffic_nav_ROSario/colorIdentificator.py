import rclpy
import cv2 as cv
import numpy as np
import signal

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32

class colorIdentificator(Node):
    def __init__(self):
        super().__init__('color_identificator')

        self.img = None
        self.bridge = CvBridge()
        self.video_writer_initialized = False
        self.out = None  # Se inicializa luego

        # Define HSV range for blue color detection
        self.lower_red1 = np.array([0, 100, 0])
        self.upper_red1 = np.array([5, 255, 255])
        self.lower_red2 = np.array([175, 60, 0])
        self.upper_red2 = np.array([180, 255, 255])
        self.lower_yellow = np.array([25, 15, 165])
        self.upper_yellow = np.array([35, 255, 255])
        self.lower_green = np.array([60, 80, 80])
        self.upper_green = np.array([80, 255, 255])
        self.lower_white = (0, 50, 180)
        self.upper_white = (92, 255, 255)

        self.N = 25
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

        self.get_logger().info('Color Identificator initialized!')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            

            # Guarda el frame
            '''if self.out is not None:
                self.out.write(self.img)'''

        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def main_loop(self):
        if self.img is None:
            self.get_logger().info('Esperando imagen...')
            return
        # Aquí vendría tu lógica de procesamiento de color...
        flip_img = cv.flip(self.img, 0)
        flip_img = cv.flip(flip_img, 1)

        alto = self.img.shape[0]

        corte = int(alto*0.45)
        flip_img_cut = flip_img[corte:,:]

        img_rgb = cv.cvtColor(flip_img_cut, cv.COLOR_BGR2RGB)
        img_hsv = cv.cvtColor(img_rgb, cv.COLOR_RGB2HSV)

        # Inicializa el video solo una vez (después de recibir el primer frame)
        if not self.video_writer_initialized:
            height, width, _ = flip_img_cut.shape
            self.out = cv.VideoWriter('output.mp4',
                                        cv.VideoWriter_fourcc(*'mp4v'),
                                        5, (width, height))
            self.video_writer_initialized = True
            self.get_logger().info('VideoWriter inicializado con resolución: {}x{}'.format(width, height))
            
        # Máscara para encontrar luces
        mask_white = cv.inRange(img_hsv, self.lower_white, self.upper_white)
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
        mask_trafic = cv.morphologyEx(mask_white, cv.MORPH_OPEN, kernel, iterations=3)
        mask_blur = cv.GaussianBlur(mask_trafic, (5, 5), 2)

        circles = cv.HoughCircles(mask_blur, cv.HOUGH_GRADIENT, dp=1.4, minDist=25,
                                param1=45, param2=22, minRadius=10, maxRadius=150)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for (x, y, r) in circles[0, :]:
                # Crear máscara para esta región circular
                circle_mask = np.zeros(mask_trafic.shape, dtype=np.uint8)
                ######################################################
                cv.circle(circle_mask, (x, y), r + ((r//10)+1), 255, -1) 
                ######################################################

                # Aplicar la máscara para cada color
                mask_red = cv.bitwise_or(
                    cv.inRange(img_hsv, self.lower_red1, self.upper_red1),
                    cv.inRange(img_hsv, self.lower_red2, self.upper_red2)
                )
                mask_red = cv.bitwise_and(mask_red, circle_mask)

                mask_yellow = cv.bitwise_and(cv.inRange(img_hsv, self.lower_yellow, self.upper_yellow), circle_mask)
                mask_green = cv.bitwise_and(cv.inRange(img_hsv, self.lower_green, self.upper_green), circle_mask)

                red_count = cv.countNonZero(mask_red)
                yellow_count = cv.countNonZero(mask_yellow)
                green_count = cv.countNonZero(mask_green)

                color_detectado = "Desconocido"
                color_bgr = (255, 255, 255)  # Blanco por defecto

                if max(red_count, yellow_count, green_count) > 50:
                    if red_count == max(red_count, yellow_count, green_count):
                        self.color_id = 1 #color_detectado = "Rojo"
                        color_bgr = (0, 0, 255)  # BGR para rojo
                    elif yellow_count == max(red_count, yellow_count, green_count):
                        self.color_id = 2 #color_detectado = "Amarillo"
                        color_bgr = (0, 255, 255)
                    elif green_count == max(red_count, yellow_count, green_count):
                        self.color_id = 3 #color_detectado = "Verde"
                        color_bgr = (0, 255, 0)
                    else:
                        self.color_id = 0

                print(f"Color detectado: {color_detectado}")

                # Dibujar el círculo en la imagen original con el color detectado
                ################################################################
                cv.circle(flip_img_cut, (x, y), r + ((r//10)+1), color_bgr, 3)
                ################################################################


        # Paso 8 : Aplicar la mascara a la imagen original para ver los colores
        #resultado = cv.bitwise_and(flip_img, flip_img, mask=color_mask)
        resultado_rgb = cv.cvtColor(mask_white, cv.COLOR_BGR2RGB)

        # Publish the cleaned binary image
        processed_img_msg = self.bridge.cv2_to_imgmsg(resultado_rgb, encoding='rgb8')
        self.out.write(flip_img_cut)
        self.get_logger().info(f"Color detectado: {self.color_id}")
        #self.image_pub.publish(processed_img_msg)
        self.color_msg.data = self.color_id
        self.color_pub.publish(self.color_msg)
    
    '''def destroy_node(self):
        # Libera el VideoWriter si fue inicializado
        if self.out is not None:
            self.out.release()
            cv.destroyAllWindows()
            self.get_logger().info('VideoWriter liberado y ventanas cerradas.')
        super().destroy_node()'''

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
    node = colorIdentificator()
    signal.signal(signal.SIGINT, node.stop_handler)

    '''try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()'''
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Nodo finalizado limpiamente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()