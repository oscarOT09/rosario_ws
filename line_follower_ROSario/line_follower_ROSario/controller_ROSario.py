# Nodo de control | Line Follower
# Equipo ROSario

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import signal

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rosario_path.msg import RosarioPath

# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('close_loop_ctrl')
        
        # Muestreo
        frecuencia_controlador = 10.0

        # Parámetros del robot
        self.max_lin_vel = 0.37
        self.min_lin_vel = 0.025
        self.max_ang_vel = 3.0
        self.min_ang_vel = 0.29

        # Delta tiempo controladores
        self.dt_pid = 1.0/frecuencia_controlador

        # PID angular
        self.declare_parameter('kp_ang_rect', 0.00001)
        self.declare_parameter('ki_ang_rect', 0.0)
        self.declare_parameter('kd_ang_rect', 0.0005)

        self.kp_ang_rect = self.get_parameter('kp_ang_rect').value # 1.7
        self.ki_ang_rect = self.get_parameter('ki_ang_rect').value # 1.2
        self.kd_ang_rect = self.get_parameter('kd_ang_rect').value # 0.2
        self.integral_ang_rect = 0.0
        self.prev_error_ang_rect = 0.0

        # PID lineal
        self.declare_parameter('kp_ang_curv', 0.003)
        self.declare_parameter('ki_ang_curv', 0.0)
        self.declare_parameter('kd_ang_curv', 0.001)

        self.kp_ang_curv = self.get_parameter('kp_ang_curv').value # 1.2
        self.ki_ang_curv = self.get_parameter('ki_ang_curv').value # 0.5
        self.kd_ang_curv = self.get_parameter('kd_ang_curv').value # 0.5
        self.integral_ang_curv = 0.0
        self.prev_error_ang_curv = 0.0

        # Bandera para actualización de los parámetros de los controladores
        self.declare_parameter('controllers_ready', False)
        self.controllers_ready = self.get_parameter('controllers_ready').value

        # Mensaje de velocidades para el Puzzlebot
        self.twist = Twist()
        # Velocidad lineal
        self.declare_parameter('linear_speed', 0.08)
        self.linear_speed = self.get_parameter('linear_speed').value # m/s
        # Velocidad angular
        self.angular_speed = 0.05  # rad/s

        # Estado de identificación de color
        self.prev_color = 0
        self.new_color = 0
        self.color_state = 0

        self.error_linea = 0
        self.curva_linea = False
        self.cont_cam = 0
        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.lineDetect_sub = self.create_subscription(RosarioPath, 'line_error', self.lineDetector_callback, 10)
        # Suscriptor para el tópico /color_id (comunicación con el nodo Color Identification)
        self.colorID_sub = self.create_subscription(Int32, 'color_id', self.colors_callback, 10)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Line Follower Navigation Controller initialized!')
        
    def lineDetector_callback(self, msg):
        if msg.curva and self.cont_cam < 10:
            self.cont_cam += 1
            self.curva_linea = True
        else:
            self.contr_cam = 0
            self.curva_linea = msg.curva
        self.error_linea = msg.error
        
        self.get_logger().info(f'Error recibido: {self.error_linea} | Curva: {self.curva_linea}')
    
    def colors_callback(self, msg):
        self.new_color = msg.data
        
        if (self.prev_color == 0 and self.new_color != 0) or (self.prev_color == 3 and (self.new_color == 2 or self.new_color == 1)) or (self.prev_color == 2 and (self.new_color == 1)) or (self.prev_color == 1 and (self.new_color == 3)):
            self.color_state = self.new_color
        else:
            self.color_state = self.prev_color
        
        self.prev_color = self.color_state
    
    def parameters_callback(self, params):
        for param in params:
            #system parameters check
            if param.name == "controllers_ready":
                #check if it is negative
                self.controllers_ready = param.value  # Update internal variable
                self.get_logger().info(f"controllers_ready updated to {self.controllers_ready}")
            elif param.name == "kp_ang_rect":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp_ang_rect = param.value  # Update internal variable
                    self.get_logger().info(f"kp_ang_rect updated to {self.kp_ang_rect}")
            elif param.name == "ki_ang_rect":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki_ang_rect = param.value  # Update internal variable
                    self.get_logger().info(f"ki_ang_rect updated to {self.ki_ang_rect}")
            elif param.name == "kd_ang_rect":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd_ang_rect = param.value  # Update internal variable
                    self.get_logger().info(f"kd_ang_rect updated to {self.kd_ang_rect}")
            
            elif param.name == "kp_ang_curv":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp_ang_curv = param.value  # Update internal variable
                    self.get_logger().info(f"kp_ang_curv updated to {self.kp_ang_curv}")
            elif param.name == "ki_ang_curv":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki_ang_curv = param.value  # Update internal variable
                    self.get_logger().info(f"ki_ang_curv updated to {self.ki_ang_curv}")
            elif param.name == "kd_ang_curv":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd_ang_curv = param.value  # Update internal variable
                    self.get_logger().info(f"kd_ang_curv updated to {self.kd_ang_curv}")
            elif param.name == "linear_speed":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.linear_speed = param.value  # Update internal variable
                    self.get_logger().info(f"linear_speed updated to {self.linear_speed}")

        return SetParametersResult(successful=True)
    
    def control_loop(self):
        if self.controllers_ready:
            if abs(self.error_linea) > 0:            
                if self.color_state == 0 or self.color_state == 3:
                    if not self.curva_linea:
                        self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(self.error_linea), self.min_ang_vel, self.max_ang_vel)                
                    else:
                        self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular_curv(self.error_linea), self.min_ang_vel, self.max_ang_vel)
                    #self.get_logger().info("Siguiendo linea")

                elif self.color_state == 2:
                    self.linear_speed = max(0.0, self.linear_speed - 0.0001)
                    self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(self.error_linea), self.min_ang_vel, self.max_ang_vel)
                    self.get_logger().info("Desacelerando por amarillo")

                elif self.color_state == 1:
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.get_logger().info('Detenido por rojo')

                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
    
    def pid_controller_angular(self, error):
        self.integral_ang_rect += error * self.dt_pid
        derivative = (error - self.prev_error_ang_rect) / self.dt_pid
        output = self.kp_ang_rect * error + self.ki_ang_rect * self.integral_ang_rect + self.kd_ang_rect * derivative
        self.prev_error_ang_rect = error
        return output

    def pid_controller_angular_curv(self, error):
        self.integral_ang_curv += error * self.dt_pid
        derivative = (error - self.prev_error_ang_curv) / self.dt_pid
        output = self.kp_ang_curv * error + self.ki_ang_curv * self.integral_ang_curv + self.kd_ang_curv * derivative
        self.prev_error_ang_curv = error
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
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

# Main
def main(args=None):#
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    signal.signal(signal.SIGINT, node.stop_handler)
    
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