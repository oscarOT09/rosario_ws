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

# Definición de la clase
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('close_loop_ctrl')
        
        # Muestreo
        frecuencia_controlador = 5.0

        # Parámetros del robot
        self.max_lin_vel = 0.0925
        self.min_lin_vel = 0.025
        self.max_ang_vel = 1.5
        self.min_ang_vel = 0.29

        # Delta tiempo controladores
        self.dt_pid = 1.0/frecuencia_controlador

        # PID angular
        self.declare_parameter('kp_ang', 0.5)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.0)

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
        self.linear_speed = 0.0925 # m/s
        # Velocidad angular
        self.angular_speed = 0.0  # rad/s

        # Estado de identificación de color
        self.prev_color = 0
        self.new_color = 0
        self.color_state = 0

        self.error_linea = 0
        
        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.lineDetect_sub = self.create_subscription(Int32, 'line_error', self.lineDetector_callback, 10)
        # Suscriptor para el tópico /color_id (comunicación con el nodo Color Identification)
        self.colorID_sub = self.create_subscription(Int32, 'color_id', self.colors_callback, 10)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Line Follower Navigation Controller initialized!')
        
    def lineDetector_callback(self, msg):
        self.error_linea = msg.data
        self.get_logger().info(f'Error recibido: {self.error_linea}')
    
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
        if self.controllers_ready and abs(self.error_linea) > 0:            
            if self.color_state == 0 or self.color_state == 3:
                '''self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.integral_lin = 0.0
                    self.prev_error_lin = 0.0
                    self.mov_state = 2'''
                self.angular_speed = self.saturate_with_deadband(self.pid_controller_angular(self.error_linea), self.min_ang_vel, self.max_ang_vel)

                '''elif self.mov_state == 2:
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.robot_busy = False'''
                
                self.get_logger().info("Siguiendo linea")

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
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
    
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