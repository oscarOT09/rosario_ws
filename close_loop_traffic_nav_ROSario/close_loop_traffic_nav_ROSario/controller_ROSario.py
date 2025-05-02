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
        
        # Muestreo
        frecuencia_controlador = 250.0 #200.0

        # Mensaje personalizado
        self.max_lin_vel = 0.0
        self.min_lin_vel = 0.0
        self.max_ang_vel = 0.0
        self.min_ang_vel = 0.0

        # Cola de almacenamiento de mensajes objetivos
        self.path_queue = []

        self.prev_pose = np.array([0.0, 0.0])
        self.curr_pose = np.array([0.0, 0.0])
        self.next_pose = np.array([0.0, 0.0])

        # Posiciones objetivo
        self.X_ref = 0.0
        self.Y_ref = 0.0

        # Referencias (entradas de los controladores)
        self.dist_ref = 0.0
        self.ang_ref = 0.0

        # Información del robot
        self.X_robot = 0.0
        self.Y_robot = 0.0
        self.Th_robot = 0.0
        
        self.dist_robot = 0.0

        self.prev_X_robot = 0.0
        self.prev_Y_robot = 0.0

        # Delta tiempo controladores
        self.dt_pid = 1.0/frecuencia_controlador

        # PID angular
        self.kp_ang = 0.2 # 0.25
        self.ki_ang = 0.0 #0.005
        self.kd_ang = 0.0 #0.002
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # PID lineal
        self.kp_lin = 0.4 #0.5
        self.ki_lin = 0.0 #0.01
        self.kd_lin = 0.0 #0.005
        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        # Mensaje de velocidades para el Puzzlebot
        self.twist = Twist()
        # Velocidad lineal
        self.linear_speed = 0.0 # m/s
        # Velocidad angular
        self.angular_speed = 1.0  # rad/s
        
        # Estado de movimiento del robot
        self.robot_busy = False

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.pose_sub = self.create_subscription(RosarioPath, 'pose', self.listener_callback, 10)
        # Suscriptor para el tópico /odom (comunicación con el nodo Localisation)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometria, qos_profile_sensor_data)

        # Estado
        self.state = 0  # 0: rotación, 1: movimiento lineal, 2: Reposo

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Close Loop Traffic Navigation Controller initialized!')

    def wrap_to_Pi(self, angle):
        result = np.fmod((angle + np.pi), (2*np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi
    
    def delta_angle(self, prev, current, next_):
        v1 = current - prev
        v2 = next_ - current

        angle1 = np.arctan2(v1[1], v1[0])
        angle2 = np.arctan2(v2[1], v2[0])

        delta = angle2 - angle1
        #delta = (delta + np.pi) % (2 * np.pi) - np.pi  # Normalización del ágnulo entre -pi y pi
        return self.wrap_to_Pi(delta)  # en radianes
    
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
        self.max_lin_vel = msg.max_lin_vel
        self.min_lin_vel = msg.min_lin_vel
        self.max_ang_vel = msg.max_ang_vel
        self.min_ang_vel = msg.min_ang_vel

        # Guardado en quque
        self.path_queue.append((new_point))
        self.get_logger().info(f'New trayectory: ({msg.path.position.x}, {msg.path.position.y})')
    
    def odometria(self, msg):
        self.X_robot = msg.pose.pose.position.x
        self.Y_robot = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quaternion)
        self.Th_robot = self.wrap_to_Pi(yaw)
    
    def control_loop(self):
        # Si el robot no está en movimiento y hay elementos faltantes en la queue, se realizan los calculos necesarios para efectuar la siguiente trayectoria 
        if not self.robot_busy and self.path_queue:
            # Siguiente objetivo
            self.next_pose = np.array(self.path_queue.pop(0))
            
            # Cálculo de referencias
            # Ángulo
            self.ang_ref += self.delta_angle(self.prev_pose, self.curr_pose, self.next_pose)
            #self.ang_ref = self.wrap_to_Pi(self.ang_ref)

            # Distancia
            dx_ref = self.next_pose[0] - self.curr_pose[0]
            dy_ref = self.next_pose[1] - self.curr_pose[1]
            self.dist_ref = np.hypot(dx_ref, dy_ref)
            self.prev_pose = self.curr_pose.copy()
            self.curr_pose = self.next_pose.copy()

            self.robot_busy = True
            self.state = 0
            self.get_logger().info(f'Objetivo: ({self.next_pose[0]}, {self.next_pose[1]}) | {np.rad2deg(self.ang_ref)} | {self.dist_ref} m')

        # Si el robot está en movimiento, se trabaja entre los estados
        if self.robot_busy:
            self.get_logger().info(f"Pose: ({self.X_robot}, {self.Y_robot}, {np.rad2deg(self.Th_robot)}°) ctrl_final")

            dx = self.next_pose[0] - self.X_robot
            dy = self.next_pose[1] - self.Y_robot

            if self.state == 0:
                
                #error_ang = self.wrap_to_Pi(self.ang_ref - self.Th_robot)
                error_ang = self.wrap_to_Pi(np.arctan2(dy,dx) - self.Th_robot)

                #self.get_logger().info(f"Error angular: {self.ang_ref}-{self.Th_robot}={np.rad2deg(error_ang):.2f}°")  
                self.get_logger().info(f"Error angular: {np.rad2deg(error_ang):.2f}°")

                if (error_ang <= np.deg2rad(0.5)) or (error_ang == 0.0): #(self.ang_ref*0.01)):
                    self.twist.angular.z = 0.0
                    
                    self.integral_ang = 0.0
                    self.prev_error_ang = 0.0
                    self.state = 1
                    
                    self.cmd_vel_pub.publish(self.twist)
                    self.get_logger().info("Rotación completada.")
                    return
                else:
                    
                    output = self.pid_controller_angular(error_ang)
                    self.twist.angular.z = self.saturate_with_deadband(output, self.min_ang_vel, self.max_ang_vel)
                    self.cmd_vel_pub.publish(self.twist)
                    return

            elif self.state == 1:
                # TRASLACIÓN
                
                
                #dx = self.X_robot - self.prev_X_robot
                #dy = self.Y_robot - self.prev_Y_robot
                #self.dist_robot += np.hypot(dx, dy)
                #self.prev_X_robot = self.X_robot
                #self.prev_Y_robot = self.Y_robot
                
                #error_lin = self.dist_ref - self.dist_robot
                error_lin = np.hypot(dx, dy) 

                #self.get_logger().info(f"Error lineal: {self.dist_ref}-{self.dist_robot}={error_lin:.3f} m")
                self.get_logger().info(f"Error lineal: {error_lin:.3f} m")

                if abs(error_lin) <= 0.048: #error_lin < 0.10:
                    self.twist.linear.x = 0.0
                    self.state = 2
                    self.dist_robot = 0.0
                    self.integral_lin = 0.0
                    self.prev_error_lin = 0.0

                    self.cmd_vel_pub.publish(self.twist)
                    self.get_logger().info("Traslación completada.")
                    return
                    
                else:
                    output = self.pid_controller_lineal(error_lin)
                    self.twist.linear.x = self.saturate_with_deadband(output, self.min_lin_vel, self.max_lin_vel)
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