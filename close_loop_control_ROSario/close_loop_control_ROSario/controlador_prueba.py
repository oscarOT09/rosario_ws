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

        self.distance_ref = 2.0
        self.distance_robot = 0.0
        self.ang_ref = np.deg2rad(90.0)

        # Bandera para identificación de movimiento del robot
        self.robot_busy = False
        
        # Variables para el controlador
        # PID motor: kp = 0.10, ki = 0.0857, kd = 0.00455
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.integral = 0.0
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
        frecuencia_odometria = 250.0 # Hz -> 0.005 s
        self.controller_timer = self.create_timer(1.0/frecuencia_controlador, self.control_loop)
        self.odometry_timer = self.create_timer(1.0/frecuencia_odometria, self.odometria)

        self.get_logger().info('Open loop controller initialized!')

    def control_loop(self):
        #self.state_start_time = self.get_clock().now()
        #now = self.get_clock().now()
        #elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        # Creación de mensaje Geometry Twist
        twist = Twist()

        # Estado de movimiento rotacional
        if self.state == 0:
            self.get_logger().info(f"Angulo Robot: {np.rad2deg(self.Th)}")
            output = self.pid_controller(self.ang_ref, self.Th)
            self.angular_speed = self.saturate_with_deadband(output, 0.29, 4.0)
            twist.angular.z = self.angular_speed
            #self.get_logger().info(f'PID ANG: {self.angular_speed}')
            
            if self.angular_speed == 0.0:
                self.state = 1
                self.get_logger().info('Rotation complete. Moving forward...')

        # Estado de movimiento lineal
        elif self.state == 1:
            output = self.pid_controller(self.distance_ref, self.distance_robot)
            self.linear_speed = self.saturate_with_deadband(output, 0.025, 0.38)
            twist.linear.x = self.linear_speed
            #self.get_logger().info(f'PID LIN: {self.angular_speed}')
            
            if self.linear_speed == 0.0:
                self.state = 2
                self.robot_busy = False
                self.linear_speed = 0.0
                self.forward_time = 0.0

                self.get_logger().info('Forward motion complete. Stopping...')
        '''        
        # Estado de reposo
        elif self.state == 2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Stopped')

            self.robot_busy = False
            self.linear_speed = 0.0
            self.forward_time = 0.0
            self.get_logger().info('Trajectory segment complete.')
        '''

        # Publicación al tópico /cmd_vel
        self.get_logger().info(f'Publishing Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
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

    def saturate_with_deadband(self, output, min_val, max_val):
        if abs(output) < min_val:
            return 0.0
        elif output > 0:
            return min(output, max_val)
        else:
            return max(output, -max_val)


    def pid_controller(self, ref, real):
        error = ref - real
        self.get_logger().info(f"Error PID {self.state}: {error}")
                               
        if self.state == 0:
            self.kp = 0.5
            self.ki = 0.01
            self.kd = 0.02
        elif self.state == 1:
            self.kp = 1.0
            self.ki = 0.05
            self.kd = 0.02

        if self.state == 0 and abs(error) <= np.deg2rad(2.5):
            output = 0.0
        elif self.state == 1 and abs(error) <= 0.02:
            output = 0.0
        else:
            self.integral += error * 0.01
            derivative = (error - self.prev_error) / 0.01

            output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

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