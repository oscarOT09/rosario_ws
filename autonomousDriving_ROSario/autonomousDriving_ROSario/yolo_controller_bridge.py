import rclpy
from rclpy.node import Node

# Custom Messages
from feedback_msg.msg import YoloResults
from action_msg.msg import YoloAction

class SignalLogger(Node):
    def __init__(self):
        super().__init__('signal_logger')

        # Subscriptor
        self.subscription = self.create_subscription(
            YoloResults,
            '/signals',
            self.listener_callback,
            10
        )

        # Publicador
        self.action_pub = self.create_publisher(
            YoloAction,
            '/action',
            10
        )

        # Timer a 30 Hz
        self.node_hz = 30.0
        timer_period = 1.0 / self.node_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Último mensaje recibido (None si aún no hay)
        self.last_msg = None

        self.get_logger().info("Nodo 'signal_logger_node' iniciado...")

    def listener_callback(self, msg: YoloResults):
        # Solo guardamos el último mensaje
        self.last_msg = msg
        self.get_logger().info(
            f"[LOG] Recibido - Traffic Light: {msg.traffic_light}, Signal: {msg.signal}, Dotted Line: {msg.dotted_line}"
        )

    def timer_callback(self):
        # Reiniciar flags cada ciclo
        parar = False
        seguir = False
        girar_r = False
        girar_l = False
        continuar = False
        desacelerar = False
        reducir = False

        # Si hay mensaje nuevo, lo procesamos
        if self.last_msg is not None:
            msg = self.last_msg
            self.last_msg = None

            # Lógica de acciones
            if msg.traffic_light == 3 or msg.signal == 1:
                parar = True
            if msg.signal == 4:
                seguir = True
            if msg.signal == 5:
                girar_r = True
            if msg.signal == 6:
                girar_l = True
            if msg.traffic_light == 1:
                continuar = True
            if msg.traffic_light == 2:
                desacelerar = True
            if msg.signal == 2 or msg.signal ==3:
                reducir = True

        # Crear y publicar mensaje
        action_msg = YoloAction()
        action_msg.parar = parar
        action_msg.seguir = seguir
        action_msg.girar_r = girar_r
        action_msg.girar_l = girar_l
        action_msg.continuar = continuar
        action_msg.desacelerar = desacelerar
        action_msg.reducir = reducir


        self.action_pub.publish(action_msg)
        self.get_logger().info("Mensaje publicado en /action")

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()