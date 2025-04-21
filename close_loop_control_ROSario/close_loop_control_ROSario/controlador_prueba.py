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

        self.dist_ref = 0.0
        # Bandera para identificación de movimiento del robot
        self.robot_busy = False
        
        # Variables para el controlador
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0

        # Variables odometria
        ## Parámetros del sistema
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0
        self._l = 0.18
        self._r = 0.05
        self._sample_time = 0.01 # s -> 10 Hz
        ## Estado interno
        self.first = True
        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0
        ## Variables para velocidad del robot
        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        ## Mensajes de recepción de los motores
        self.wr = Float32()
        self.wl = Float32()
        
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
        frecuencia_controlador = 100.0 # Hz -> 0.01 s
        # self.timer_period = 0.1 # 10 Hz 
        frecuencia_odometria = 200.0 # Hz -> 0.005 s
        self.controller_timer = self.create_timer(1.0/frecuencia_controlador, self.control_loop)
        self.odometry_timer = self.create_timer(1.0/frecuencia_odometria, self.odometria)

        self.get_logger().info('Open loop controller initialized!')

    def control_loop(self):
        self.state_start_time = self.get_clock().now()
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
            self.linear_speed = self.pid_controller(self.distance_ref, self.distance_robot)
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
    
    def odometria(self):
        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.current_time = self.start_time
            self.first = False
            return
        
        """ Updates robot position based on real elapsed time """
        # Get current time and compute dt
        current_time = self.get_clock().now()
        self.dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        
        if self.dt > self._sample_time:
            #Wheel Tangential Velocities
            self.v_r = self._r  * self.wr.data
            self.v_l = self._r  * self.wl.data

            #Robot Velocities
            self.V = (1/2.0) * (self.v_r + self.v_l)
            self.Omega = (1.0/self._l) * (self.v_r - self.v_l)

            # Robot position in x
            self.X += self.V * np.cos(self.Th) * self.dt
            # Robot position in y
            self.Y += self.V * np.sin(self.Th) * self.dt
            # Robot theta
            self.Th += self.Omega * self.dt

            # Distancia total recorrida por el robot
            self.distance_robot += abs(self.V) * self.dt

            self.last_time = current_time

    def encR_callback(self, msg):
        self.wr = msg

    def encL_callback(self, msg):
        self.wl = msg

    def pid_controller(self, ref, real):
        error = ref - real
        
        integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

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