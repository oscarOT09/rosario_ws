# Nodo controlador | Etapa 2 - Challenge 3

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d

from geometry_msgs.msg import Twist
#from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry # from rosario_path.msg import RosarioPath
from rclpy.qos import qos_profile_sensor_data


# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        self.X_ref = 1.0
        self.Y_ref = 1.0

        self.pose_robot = 0.0
        
        self.ang_ref = 0.0

        # Bandera para identificación de movimiento del robot
        self.robot_busy = True
        
        # Variables para el controlador
        # PID motor: kp = 0.10, ki = 0.0857, kd = 0.00455
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        self.integral_lin = 0.0
        self.prev_error_lin = 0.0
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # Variables odometria
        self.X_robot = 0.0
        self.Y_robot = 0.0
        self.Th_robot = 0.0
        self.V_robot = 0.0
        self.Omega_Robot = 0.0

        self.odom_msg = Odometry()
        self.twist = Twist()
        
        # Queue poses
        self.path_queue = []

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        ### SUSCRIPCION A ODOM
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometria, qos_profile_sensor_data)
        # Velocidad lineal (valor dependiente del valor enviado por el nodo Path)
        self.linear_speed = 0.0 # m/s

        # Velocidad angular (constante para asegurar el movimiento constante del robot)
        self.angular_speed = 0.0  # rad/s

        # Estado
        self.state = 0  # 0: rotación, 1: movimiento lineal, 2: Reposo
        
        # Frecuencia de muestreo
        frecuencia_controlador = 200.0 # Hz -> 0.01 s
        # self.timer_period = 0.1 # 10 Hz 
        self.controller_timer = self.create_timer(1.0/frecuencia_controlador, self.control_loop)

        self.get_logger().info('Open loop controller initialized!')

    def control_loop(self):
        if self.robot_busy:
            if self.state == 0:
                ###OBTENER EL ANGULO ENTRE EL ROBOT Y EL OBJETIVO
                # Calcular ángulo objetivo entre posición actual y objetivo
                dx = self.X_ref - self.X_robot
                dy = self.Y_ref - self.Y_robot
                self.ang_ref = np.arctan2(dy, dx)  # ángulo deseado en radianes
                self.get_logger().info(f"Angulo ref: {np.rad2deg(self.ang_ref)}°")

                # Estado de movimiento rotacional
                output_angular = self.pid_controller_angular(self.ang_ref, self.Th_robot)
                self.angular_speed = self.saturate_with_deadband(output_angular, 0.29, 3.0)
                self.twist.angular.z = self.angular_speed
                #self.get_logger().info(f'PID ANG: {self.angular_speed}')
                if self.angular_speed == 0.0:
                    self.state = 1
                    self.get_logger().info('Angle achieved.')    
            
            elif self.state == 1:
                ###CALCULAR LA DISTANCIA ENTRE PUNTOS
                cords_robot = [(self.X_robot), (self.Y_robot)]
                cords_obj = [(self.X_ref), (self.Y_ref)]

                # Estado de movimiento lineal
                output_lineal = self.pid_controller_lineal(cords_obj, cords_robot)
                self.linear_speed = self.saturate_with_deadband(output_lineal, 0.025, 0.38)
                self.twist.linear.x = self.linear_speed
                #self.get_logger().info(f'PID LIN: {self.angular_speed}')
                if self.linear_speed == 0.0:
                    self.state = 2
                    self.get_logger().info('Distance achieved.')

            elif self.state == 2:
                self.robot_busy = False   
                self.get_logger().info('Objective achieved. Stopping...')

            # Publicación al tópico /cmd_vel
            #self.get_logger().info(f'Publishing Twist: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}')
            self.cmd_vel_pub.publish(self.twist)
    
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))

        if(result < 0):
            result += 2*np.pi
        
        return result - np.pi

    def odometria(self, msg):
        self.odom_msg = msg

        self.X_robot = msg.pose.pose.position.x
        self.Y_robot = msg.pose.pose.position.y
        #self.Th_robot = msg.pose.pose.orientation.z

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)
        self.Th_robot = self.wrap_to_Pi(yaw)
        self.get_logger().info(f"Angulo recibido: {np.rad2deg(self.Th_robot)}")

        self.V_robot = msg.twist.twist.linear.x
        self.Omega_Robot = msg.twist.twist.angular.z 

    def saturate_with_deadband(self, output, min_val, max_val):
        if abs(output) < min_val:
            return 0.0
        elif output > 0:
            return min(output, max_val)
        else:
            return max(output, -max_val)

    def pid_controller_lineal(self, ref, real):
        error = np.linalg.norm(np.array(ref) - np.array(real)) # ref - real
        self.get_logger().info(f"Error PID Lineal: {error}")
        
        self.kp = 1.0
        self.ki = 0.05
        self.kd = 0.02

        if abs(error) <= 0.02:
            output = 0.0
        else:
            self.integral_lin += error * 0.01
            derivative = (error - self.prev_error_lin) / 0.01

            output = (self.kp * error) + (self.ki * self.integral_lin) + (self.kd * derivative)

        self.prev_error = error

        return output
    
    def pid_controller_angular(self, ref, real):
        error = self.wrap_to_Pi(ref - real)
        self.get_logger().info(f"Error PID Angular: {np.rad2deg(error)}")
                               
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.02

        if abs(error) <= np.deg2rad(2.5):
            output = 0.0
        else:
            self.integral_ang += error * 0.01
            derivative = (error - self.prev_error_ang) / 0.01

            output = (self.kp * error) + (self.ki * self.integral_ang) + (self.kd * derivative)

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