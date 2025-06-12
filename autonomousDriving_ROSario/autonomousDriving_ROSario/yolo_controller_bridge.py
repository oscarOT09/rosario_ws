# Nodo puente YOLO-Controlador | Final-Term Challenge
# Equipo ROSario

# Importaciones necesarias
import rclpy
from rclpy.node import Node

# Custom Messages
from yolo_msg.msg import Yolov8Inference, InferenceResult
from action_msg.msg import YoloAction

class SignalLogger(Node):
    def __init__(self):
        super().__init__('signal_logger')

        # Suscripción al tópico de detecciones YOLO
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_inference',
            self.listener_callback,
            10
        )

        # Publicador de acciones interpretadas
        self.action_pub = self.create_publisher(
            YoloAction,
            '/action',
            10
        )

        # Parametros Dinamicos
        self.declare_parameter('min_signal_area', 5300)
        self.declare_parameter('min_traffic_area', 1500)

        # Frecuencia de publicación en Hz
        self.node_hz = 10.0
        self.timer = self.create_timer(1.0 / self.node_hz, self.timer_callback)

        # Variable que guarda las detecciones más recientes
        self.last_detections = []

        self.get_logger().info("Nodo 'signal_logger_node' iniciado...")

    def listener_callback(self, msg):
        # Guardar la lista de objetos detectados del mensaje entrante
        self.last_detections = msg.yolov8_inference

    def timer_callback(self):
        # Crear mensaje de acción (False)
        action_msg = YoloAction()

        # Si no hay detecciones, publicar mensaje vacío
        if not self.last_detections:
            #self.get_logger().info("Sin detecciones: enviando acción nula.")
            self.action_pub.publish(action_msg)
            return
        
        # Mapeo de clases
        SIGNALS = {
            "stop": 1,
            "roadwork_ahead": 2,
            "give_way": 3,
            "ahead_only": 4,
            "turn_right": 5,
            "turn_left": 6,
        }

        TRAFFIC_LIGHTS = {
            "trafficlight_green": 1,
            "trafficlight_yellow": 2,
            "trafficlight_red": 3,
        }

        # Obtencion de nuevos cambios en los parametros
        min_signal_area = self.get_parameter('min_signal_area').get_parameter_value().integer_value
        min_traffic_area = self.get_parameter('min_traffic_area').get_parameter_value().integer_value

        # Calculo del area
        max_signal_area = 0
        selected_signal = 0

        max_traffic_area = 0
        selected_traffic = 0

        for det in self.last_detections:
            cls = det.class_name.lower()
            w = det.right - det.left
            h = det.bottom - det.top
            area = w * h
            accur = det.accuracy
            
            
            if accur < 0.5:
                continue

            #self.get_logger().info(f'Recibido: {cls} con accur: {accur}')
            self.get_logger().info(f'Recibido: {cls} con area: {area}')
            if cls in SIGNALS:
                if area > max_signal_area:
                    max_signal_area = area
                    selected_signal = SIGNALS[cls]
            elif cls in TRAFFIC_LIGHTS:
                if area > max_traffic_area:
                    max_traffic_area = area
                    selected_traffic = TRAFFIC_LIGHTS[cls]

        # Aplicar los filtros por área mínima
        final_signal = selected_signal if max_signal_area >= min_signal_area else 0
        final_traffic = selected_traffic if max_traffic_area >= min_traffic_area else 0

        # Llenar el mensaje de acción según los resultados
        action_msg.alto = final_signal == 1
        action_msg.trabajo = final_signal == 2
        action_msg.ceder = final_signal == 3
        action_msg.adelante = final_signal == 4
        action_msg.girar_r = final_signal == 5
        action_msg.girar_l = final_signal == 6
        action_msg.verde = final_traffic == 1
        action_msg.amarillo = final_traffic == 2
        action_msg.rojo = final_traffic == 3

        self.action_pub.publish(action_msg)

        # Logs de Debuggeo
        if final_signal:
            name = list(SIGNALS.keys())[list(SIGNALS.values()).index(final_signal)]
            #self.get_logger().info(f"Señal seleccionada: {name} (área {max_signal_area})")
        if final_traffic:
            name = list(TRAFFIC_LIGHTS.keys())[list(TRAFFIC_LIGHTS.values()).index(final_traffic)]
            #self.get_logger().info(f"Semáforo seleccionado: {name} (área {max_traffic_area})")

        # Reiniciar detecciones tras publicarlas (opcional)
        self.last_detections = []

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()