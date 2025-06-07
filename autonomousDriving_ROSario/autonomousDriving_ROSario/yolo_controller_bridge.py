import rclpy
from rclpy.node import Node
from yolo_msg.msg import Yolov8Inference, InferenceResult
from action_msg.msg import YoloAction

class SignalLogger(Node):
    def __init__(self):
        super().__init__('signal_logger')

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_inference',
            self.listener_callback,
            10
        )

        self.action_pub = self.create_publisher(
            YoloAction,
            '/action',
            10
        )

        self.node_hz = 15.0
        self.timer = self.create_timer(1.0 / self.node_hz, self.timer_callback)
        self.last_detections = []

        self.get_logger().info("Nodo 'signal_logger_node' iniciado...")

    def listener_callback(self, msg):
        self.last_detections = msg.yolov8_inference

    def timer_callback(self):
        if not self.last_detections:
            action_msg = YoloAction()
            action_msg.alto = False
            action_msg.trabajo = False
            action_msg.ceder = False
            action_msg.adelante = False
            action_msg.girar_r = False
            action_msg.girar_l = False
            action_msg.verde = False
            action_msg.amarillo = False
            action_msg.rojo = False

            self.action_pub.publish(action_msg)
            self.get_logger().info("Sin detecciones: acción 0 enviada")
            return

        # Inicializamos señales
        traffic_light = 0
        signal = 0
        dotted_line = False

        max_signal_area = 0
        max_traffic_area = 0

        SIGNALS = {
            "stop": 1,
            "roadwork_ahead": 2,
            "give_way": 3,
            "ahead_only": 4,
            "turn_right": 5,
            "turn_left": 6,
        }

        TRAFFIC_LIGHTS = {
            "trafficLight_green": 1,
            "trafficLight_yellow": 2,
            "trafficLight_red": 3,
        }

        for det in self.last_detections:
            w = det.right - det.left
            h = det.bottom - det.top
            area = w * h
            cls = det.class_name.lower()

            if cls in SIGNALS:
                if area > max_signal_area:
                    signal = SIGNALS[cls]
                    max_signal_area = area

            elif cls in TRAFFIC_LIGHTS:
                if area > max_traffic_area:
                    traffic_light = TRAFFIC_LIGHTS[cls]
                    max_traffic_area = area

            elif "line" in cls or "dotted" in cls:
                dotted_line = True


        # Lógica de acción binaria
        alto = signal == 1
        trabajo = signal == 2
        ceder = signal == 3
        adelante = signal == 4
        girar_r = signal == 5
        girar_l = signal == 6
        verde = traffic_light == 1
        amarillo = traffic_light == 2
        rojo = traffic_light == 3
        if traffic_light != 0:
            semaforo = True

        action_msg = YoloAction()
        action_msg.alto = alto
        action_msg.trabajo = trabajo
        action_msg.ceder = ceder
        action_msg.adelante = adelante
        action_msg.girar_r = girar_r
        action_msg.girar_l = girar_l
        action_msg.verde = verde
        action_msg.amarillo = amarillo
        action_msg.rojo = rojo

        self.action_pub.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
