# Nodo de control | Challenge 3

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosario_path.msg import RosarioPath

# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('close_loop_ctrl')
        
        self.factor = 1.05

        self.factor_dist = 1.09
        self.factor_ang = 0.8
        
        # Posiciones objetivo
        self.X_ref = 0.0
        self.Y_ref = 0.0
        self.dist_ref = 0.0
        self.ang_ref = 0.0

        # Estados actuales
        self.X_robot = 0.0
        self.Y_robot = 0.0
        self.Th_robot = 0.0
        
        self.dist_robot = 0.0

        self.prev_X_robot = 0.0
        self.prev_Y_robot = 0.0

        self.prev_goal_X = 0.0
        self.prev_goal_Y = 0.0

        # PID angular
        self.kp_ang = 0.25
        self.ki_ang = 0.0 #0.005
        self.kd_ang = 0.0 #0.002
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # PID lineal
        self.kp_lin = 0.5
        self.ki_lin = 0.0 #0.01
        self.kd_lin = 0.0 #0.005
        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        # Velocidades
        self.twist = Twist()
        
        # Bandera para identificación de movimiento del robot
        self.robot_busy = False
        
        # Cola de almacenamiento de mensajes RosarioPath
        self.path_queue = []

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor para el tópico /pose
        self.pose_sub = self.create_subscription(RosarioPath, 'pose', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometria, qos_profile_sensor_data)

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
        
        # Muestreo
        frecuencia_controlador = 200.0
        self.dt_pid = 1.0/200.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Close loop controller initialized!')

    def wrap_to_Pi(self, angle):
        result = np.fmod((angle + np.pi), (2*np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi
    
    def pid_controller_angular(self, error):
        self.integral_ang += error * self.dt_pid
        derivative = (error - self.prev_error_ang) / self.dt_pid
        output = self.kp_ang * error + self.ki_ang * self.integral_ang + self.kd_ang * derivative
        self.prev_error_ang = error
        return output

    def pid_controller_lineal(self, error):
        self.integral_lin += error * self.dt_pid
        derivative = (error - self.prev_error_lin) / self.dt_pid
        output = self.kp_lin * error + self.ki_lin * self.integral_lin + self.kd_lin * derivative
        self.prev_error_lin = error
        return output

    def saturate_with_deadband(self, output, min_val, max_val):
        if abs(output) < min_val:
            return min_val
        elif output > 0:
            return min(output,max_val)
        else:
            return max(output, -max_val)
        
    def listener_callback(self, msg):
        # Descomposición para almacenamiento local en variables 
        new_point = np.array([msg.path.position.x, msg.path.position.y])
        velocity = msg.velocity
        duration = msg.time

        # Guardado en quque
        self.path_queue.append((new_point))
        self.get_logger().info(f'New trayectory: ({msg.path.position.x}, {msg.path.position.y})')
    
    def odometria(self, msg):
        self.X_robot = msg.pose.pose.position.x
        self.Y_robot = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quaternion)
        self.Th_robot = self.wrap_to_Pi(yaw*self.factor) 

    def control_loop(self):
        # Si el robot no está en movimiento y hay elementos faltantes en la queue, se realizan los calculos necesarios para efectuar la siguiente trayectoria 
        if not self.robot_busy and self.path_queue:
            self.X_ref, self.Y_ref = self.path_queue.pop(0)
            
            dx = self.X_ref - self.prev_goal_X
            dy = self.Y_ref - self.prev_goal_Y
            self.ang_ref = np.arctan2(dy, dx) * self.factor_ang

            self.dist_ref = np.hypot(dx, dy)

            self.prev_goal_X = self.X_robot
            self.prev_goal_Y = self.Y_robot

            self.robot_busy = True
            self.state = 0
            self.get_logger().info(f'Nuevo objetivo: ({self.X_ref}, {self.Y_ref})')

        # Si el robot está en movimiento, se trabaja entre los estados
        if self.robot_busy:
            dx = self.X_ref - self.X_robot
            dy = self.Y_ref - self.Y_robot

            if self.state == 0:
                
                error = self.wrap_to_Pi(self.ang_ref - self.Th_robot)
                
                self.get_logger().info(f"Error angular: {self.ang_ref}-{self.Th_robot}={np.rad2deg(error):.2f}°")

                if abs(error) < np.deg2rad(10.0):
                    self.twist.angular.z = 0.0
                    
                    self.integral_ang = 0.0
                    self.prev_error_ang = 0.0
                    self.factor += 0.125
                    self.state = 1
                    
                    self.cmd_vel_pub.publish(self.twist)
                    self.get_logger().info("Rotación completada.")
                    return
                else:
                    
                    output = self.pid_controller_angular(error)
                    self.twist.angular.z = self.saturate_with_deadband(output, 0.29, 3.0)
                    self.cmd_vel_pub.publish(self.twist)
                    return

            elif self.state == 1:
                # TRASLACIÓN
                self.dist_robot += np.linalg.norm([self.X_robot - self.prev_X_robot, self.Y_robot - self.prev_Y_robot]) * self.factor_dist
                error = self.dist_ref - self.dist_robot

                self.get_logger().info(f"Error lineal: {error:.3f} m")
                self.prev_X_robot = self.X_robot
                self.prev_Y_robot = self.Y_robot

                if error < 0.1:
                    self.twist.linear.x = 0.0
                    self.state = 2
                    self.dist_robot = 0.0
                    self.integral_lin = 0.0
                    self.prev_error_lin = 0.0

                    self.factor_dist += self.dist_robot*0.005

                    self.cmd_vel_pub.publish(self.twist)
                    self.get_logger().info("Traslación completada.")
                    return
                    
                else:
                    output = self.pid_controller_lineal(error)
                    self.twist.linear.x = self.saturate_with_deadband(output, 0.025, 0.38)
                    self.cmd_vel_pub.publish(self.twist)
                    return

            elif self.state == 2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

                self.robot_busy = False
                self.cmd_vel_pub.publish(self.twist)
                self.get_logger().info("Objetivo alcanzado. Esperando siguiente...")
                

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