# Nodo de control | Mid-term Challenge

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d
import signal

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
from rosario_path.msg import RosarioPath

# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('close_loop_ctrl')
        
        # Muestreo
        frecuencia_controlador = 250.0

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

        self.prev_X_robot = 0.0
        self.prev_Y_robot = 0.0

        # Delta tiempo controladores
        self.dt_pid = 1.0/frecuencia_controlador

        # PID angular
        self.declare_parameter('kp_ang', 1.7)
        self.declare_parameter('ki_ang', 1.2)
        self.declare_parameter('kd_ang', 0.2)

        self.kp_ang = self.get_parameter('kp_ang').value # 1.7
        self.ki_ang = self.get_parameter('ki_ang').value # 1.2
        self.kd_ang = self.get_parameter('kd_ang').value # 0.2
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # PID lineal
        self.declare_parameter('kp_lin', 1.2)
        self.declare_parameter('ki_lin', 0.5)
        self.declare_parameter('kd_lin', 0.5)

        self.kp_lin = self.get_parameter('kp_lin').value # 1.2
        self.ki_lin = self.get_parameter('ki_lin').value # 0.5
        self.kd_lin = self.get_parameter('kd_lin').value # 0.5
        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        # Bandera para actualización de los parámetros de los controladores
        self.declare_parameter('controllers_ready', False)
        self.controllers_ready = self.get_parameter('controllers_ready').value

        # Mensaje de velocidades para el Puzzlebot
        self.twist = Twist()
        # Velocidad lineal
        self.linear_speed = 0.0 # m/s
        # Velocidad angular
        self.angular_speed = 1.0  # rad/s
        
        # Estado de movimiento del robot
        self.robot_busy = False

        # Estado de identificación de color
        self.color_state = 0

        # Estado
        self.mov_state = 0  # 0: rotación, 1: movimiento lineal, 2: Reposo

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.pose_sub = self.create_subscription(RosarioPath, 'pose', self.goals_callback, 10)
        # Suscriptor para el tópico /odom (comunicación con el nodo Localisation)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometriaPuzzlebot_callback, qos_profile_sensor_data)
        # Suscriptor para el tópico /color_id (comunicación con el nodo Color Identification)
        self.colorID_sub = self.create_subscription(Int32, 'color_id', self.colors_callback, 10)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Close Loop Traffic Navigation Controller initialized!')
        
    def goals_callback(self, msg):
        # Descomposición para almacenamiento local en variables 
        new_point = np.array([msg.path.position.x, msg.path.position.y])
        self.max_lin_vel = msg.max_lin_vel
        self.min_lin_vel = msg.min_lin_vel
        self.max_ang_vel = msg.max_ang_vel
        self.min_ang_vel = msg.min_ang_vel

        # Guardado en queue
        self.path_queue.append((new_point))
        self.get_logger().info(f'New trayectory: ({msg.path.position.x}, {msg.path.position.y})')
    
    def odometriaPuzzlebot_callback(self, msg):
        self.X_robot = msg.pose.pose.position.x
        self.Y_robot = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quaternion)
        self.Th_robot = self.wrap_to_Pi(yaw)
    
    def colors_callback(self, msg):
        self.color_state = msg.data
    
    def parameters_callback(self, params):
        for param in params:
            #system parameters check
            if param.name == "controllers_ready":
                #check if it is negative
                self.controllers_ready = param.value  # Update internal variable
                self.get_logger().info(f"controllers_ready updated to {self.controllers_ready}")
            elif param.name == "kp_ang":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp_ang = param.value  # Update internal variable
                    self.get_logger().info(f"kp_ang updated to {self.kp_ang}")
            elif param.name == "ki_ang":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki_ang = param.value  # Update internal variable
                    self.get_logger().info(f"ki_ang updated to {self.ki_ang}")
            elif param.name == "kd_ang":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd_ang = param.value  # Update internal variable
                    self.get_logger().info(f"kd_ang updated to {self.kd_ang}")
            
            elif param.name == "kp_lin":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp_lin = param.value  # Update internal variable
                    self.get_logger().info(f"kp_lin updated to {self.kp_lin}")
            elif param.name == "ki_lin":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki_lin = param.value  # Update internal variable
                    self.get_logger().info(f"ki_lin updated to {self.ki_lin}")
            elif param.name == "kd_lin":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd_lin = param.value  # Update internal variable
                    self.get_logger().info(f"kd_lin updated to {self.kd_lin}")
        return SetParametersResult(successful=True)
    
    def control_loop(self):
        # Si el robot no está en movimiento y hay elementos faltantes en la queue, se realizan los calculos necesarios para efectuar la siguiente trayectoria 
        if not self.robot_busy and self.path_queue:
            # Siguiente objetivo
            self.next_pose = np.array(self.path_queue.pop(0))
            
            # Cálculo de referencias
            # Ángulo
            self.ang_ref += self.delta_angle(self.prev_pose, self.curr_pose, self.next_pose)

            # Distancia
            dx_ref = self.next_pose[0] - self.curr_pose[0]
            dy_ref = self.next_pose[1] - self.curr_pose[1]
            self.dist_ref = np.hypot(dx_ref, dy_ref)
            
            self.prev_pose = self.curr_pose.copy()
            self.curr_pose = self.next_pose.copy()

            self.robot_busy = True
            self.mov_state = 0
            self.get_logger().info(f'Objetivo: ({self.next_pose[0]}, {self.next_pose[1]}) | {np.rad2deg(self.ang_ref)} | {self.dist_ref} m')

        # Si el robot está en movimiento, se trabaja entre los estados
        if self.robot_busy and self.controllers_ready:
            self.get_logger().info(f"Pose: ({self.X_robot}, {self.Y_robot}, {np.rad2deg(self.Th_robot)}°) | Color ID: {self.color_state}")
            
            dx = self.next_pose[0] - self.X_robot
            dy = self.next_pose[1] - self.Y_robot

            if self.color_state == 0 or self.color_state == 3:
                if self.mov_state == 0: # ROTACIÓN
                    error_ang = self.wrap_to_Pi(np.arctan2(dy,dx) - self.Th_robot)
                    self.get_logger().info(f"Error angular: {np.rad2deg(error_ang):.2f}°")

                    if (abs(error_ang) <= np.deg2rad(3.0)) or (error_ang == 0.0):
                        self.angular_speed = 0.0
                        self.integral_ang = 0.0
                        self.prev_error_ang = 0.0
                        self.mov_state = 1
                    else:
                        self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(error_ang), self.min_ang_vel, self.max_ang_vel)

                elif self.mov_state == 1: # TRASLACIÓN
                    error_lin = np.hypot(dx, dy)
                    error_ang = self.wrap_to_Pi(np.arctan2(dy,dx) - self.Th_robot)
                    self.get_logger().info(f"Error lineal: {error_lin} m | Error angular: {np.rad2deg(error_ang):.2f}°")

                    if abs(error_lin) <= 0.001:
                        self.linear_speed = 0.0
                        self.angular_speed = 0.0
                        self.integral_lin = 0.0
                        self.prev_error_lin = 0.0
                        self.mov_state = 2                        
                    else:
                        self.linear_speed = self.saturate_with_deadband(self.pid_controller_lineal(error_lin), self.min_lin_vel, self.max_lin_vel)
                        self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(error_ang), self.min_ang_vel, self.max_ang_vel)

                elif self.mov_state == 2:
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.robot_busy = False
                    self.get_logger().info("Objetivo alcanzado. Esperando siguiente...")
                    self.get_logger().info("------------------------------------------------------------------------------------------")
                
                '''self.twist.linear.x = self.linear_speed
                self.twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(self.twist)'''

            elif self.color_state == 2:
                if self.mov_state == 0: # ROTACIÓN
                    error_ang = self.wrap_to_Pi(np.arctan2(dy,dx) - self.Th_robot)
                    self.get_logger().info(f"Error angular: {np.rad2deg(error_ang):.2f}°")

                    if (abs(error_ang) <= np.deg2rad(2.8)) or (error_ang == 0.0):
                        self.angular_speed = 0.0
                        self.integral_ang = 0.0
                        self.prev_error_ang = 0.0
                        self.mov_state = 1
                    else:
                        self.angular_speed = max(0.0, self.angular_speed - 0.0001)

                elif self.mov_state == 1: # TRASLACIÓN
                    error_lin = np.hypot(dx, dy)
                    error_ang = self.wrap_to_Pi(np.arctan2(dy,dx) - self.Th_robot)
                    self.get_logger().info(f"Error lineal: {error_lin} m | Error angular: {np.rad2deg(error_ang):.2f}°")

                    if abs(error_lin) <= 0.05:
                        self.linear_speed = 0.0
                        self.angular_speed = 0.0
                        self.integral_lin = 0.0
                        self.prev_error_lin = 0.0
                        self.mov_state = 2
                    else:
                        self.linear_speed = max(0.0, self.linear_speed - 0.0001)
                        self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(error_ang), self.min_ang_vel, self.max_ang_vel)

                elif self.mov_state == 2:
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.robot_busy = False

                    self.get_logger().info("Objetivo alcanzado. Esperando siguiente...")
                    self.get_logger().info("------------------------------------------------------------------------------------------")
                
                '''self.twist.linear.x = self.linear_speed
                self.twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(self.twist)'''
            
            elif self.color_state == 1:
                #self.mov_state = 0
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                '''self.twist.linear.x = self.linear_speed
                self.twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(self.twist)'''
                self.get_logger().info('Detenido por ver rojo')

            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.twist)

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
        if output > 0:
            if abs(output) < min_val:
                return min_val
            else:
                return min(output,max_val)
        else:
            if abs(output) < min_val:
                return -min_val
            else:
                return max(output, -max_val)
    
    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

# Main
def main(args=None):#
    rclpy.init(args=args)
    node = OpenLoopCtrl()
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

# Execute
if __name__ == '__main__':
    main()