# Nodo controlador | Etapa 2 - Challenge 3

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
#from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
from rosario_path.msg import RosarioPath
from rclpy import qos


# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        # Bandera para identificación de movimiento del robot
        self.robot_busy = False
        
        # Cola de almacenamiento de mensajes RosarioPath
        self.path_queue = []

        # Inicialización de posiciones
        self.prev_pose = np.array([0.0, 0.0])
        self.curr_pose = np.array([0.0, 0.0])
        self.next_pose = np.array([0.0, 0.0])

        # Variables para el controlador
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscripciones
        self.pose_sub = self.create_subscription(RosarioPath, 'pose', self.path_callback, 10)
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)

        # Velocidad lineal (valor dependiente del valor enviado por el nodo Path)
        self.linear_speed = 0.0 # m/s

        # Velocidad angular (constante para asegurar el movimiento constante del robot)
        self.angular_speed = 1.0  # rad/s

        # Tiempo de movimiento lineal (valor dependiente del valor enviado por el nodo Path)
        self.forward_time = 0.0

        # Tiempo de movimiento rotacional (calculado en control_loop)
        self.rotate_time = 0.0

        # Estado
        self.state = 0  # 0: rotación, 1: movimiento lineal, 2: Reposo
        
        # Frecuencia de muestreo
        self.timer_period = 0.1 # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Open loop controller initialized!')

    def delta_angle(self, prev, current, next_):
        v1 = current - prev
        v2 = next_ - current

        angle1 = np.arctan2(v1[1], v1[0])
        angle2 = np.arctan2(v2[1], v2[0])

        delta = angle2 - angle1
        delta = (delta + np.pi) % (2 * np.pi) - np.pi  # Normalización del ágnulo entre -pi y pi
        return delta  # en radianes

    def path_callback(self, msg):
        # Descomposición para almacenamiento local en variables 
        new_point = np.array([msg.path.position.x, msg.path.position.y])
        velocity = msg.velocity
        duration = msg.time

        # Guardado en quque
        self.path_queue.append((new_point, velocity, duration))
        self.get_logger().info(f'New trayectory: ({msg.path.position.x}, {msg.path.position.y})')

    def control_loop(self):
        # Si el robot no está en movimiento y hay elementos faltantes en la queue, se realizan los calculos necesarios para efectuar la siguiente trayectoria 
        if not self.robot_busy and self.path_queue:
            # Actualziación de posiciones de referencia
            self.prev_pose = self.curr_pose.copy()
            self.curr_pose = self.next_pose.copy()
            self.next_pose, self.linear_speed, self.forward_time = self.path_queue.pop(0)

            # Calculo del ángulo relativo
            self.delta_theta = self.delta_angle(self.prev_pose, self.curr_pose, self.next_pose)
            # Calculo de tiempo de rotación en base al nuevo ángulo relativo
            self.rotate_time = round(abs(self.delta_theta)*0.85, 2) / self.angular_speed
            # Determinación del ángulo de giro
            self.rotation_direction = np.sign(self.delta_theta)

            # Inicio en estado 0
            self.state = 0
            # Cambio de bandera para indicar que el robot está en movimiento
            self.robot_busy = True

            self.state_start_time = self.get_clock().now()

        # Si el robot está en movimiento, se trabaja entre los estados
        if self.robot_busy:
            now = self.get_clock().now()
            elapsed = (now - self.state_start_time).nanoseconds * 1e-9

            # Creación de mensaje Geometry Twist
            twist = Twist()

            # Estado de movimiento rotacional
            if self.state == 0:
                twist.angular.z = self.rotation_direction * self.angular_speed
                self.get_logger().info(f'Rotating {np.rad2deg(self.delta_theta)}°, {self.rotate_time} s')
                
                if elapsed >= self.rotate_time:
                    self.state = 1
                    self.state_start_time = now
                    self.get_logger().info('Rotation complete. Moving forward...')

            # Estado de movimiento lineal
            elif self.state == 1:
                self.linear_speed = self.pid_controller(pos_ref, pos_robot)
                twist.linear.x = self.linear_speed
                self.get_logger().info(f'Moving {self.forward_time} at {self.linear_speed} m/s')
                
                if elapsed >= self.forward_time:
                    self.state = 2
                    self.state_start_time = now
                    self.get_logger().info('Forward motion complete. Stopping...')
            
            # Estado de reposo
            elif self.state == 2:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Stopped')

                self.robot_busy = False
                self.linear_speed = 0.0
                self.forward_time = 0.0
                self.get_logger().info('Trajectory segment complete.')

            # Publicación al tópico /cmd_vel
            self.cmd_vel_pub.publish(twist)
    
    def encR_callback(self, msg):
        self.wr = msg

    def encL_callback(self, msg):
        self.wl = msg

    def pid_controller(self, ref, real):
        error = ref - real
        
        integral = 0.0
        derivative = 0.0

        output = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)

        self.prev_error = error

        return output

# Main
def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Execute
if __name__ == '__main__':
    main()