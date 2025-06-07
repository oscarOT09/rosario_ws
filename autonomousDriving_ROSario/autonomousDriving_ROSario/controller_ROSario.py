# Nodo de control | Line Follower
# Equipo ROSario

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import signal

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from action_msg.msg import YoloAction

# Definición de la clase
class trafficNavController(Node):
    def __init__(self):
        super().__init__('close_loop_ctrl')
        
        # Muestreo
        frecuencia_controlador = 15.0

        self.declare_parameter('init_time', 30)
        self.init_time = self.get_parameter('init_time').value

        self.declare_parameter('max_ang_vel', 2.5)
        self.declare_parameter('min_ang_vel', 0.31)

        # Parámetros del robot
        self.max_lin_vel = 0.37
        self.min_lin_vel = 0.025
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.min_ang_vel = self.get_parameter('min_ang_vel').value

        # Delta tiempo controladores
        self.dt_pid = 1.0/frecuencia_controlador

        # PID
        self.declare_parameter('kp_ang_curv', 0.55)
        self.declare_parameter('ki_ang_curv', 0.001)
        self.declare_parameter('kd_ang_curv', 0.11)

        self.kp_ang_curv = self.get_parameter('kp_ang_curv').value # 1.2
        self.ki_ang_curv = self.get_parameter('ki_ang_curv').value # 0.5
        self.kd_ang_curv = self.get_parameter('kd_ang_curv').value # 0.5
        self.integral_ang_curv = 0.0
        self.prev_error_ang_curv = 0.0

        # Bandera para actualización de los parámetros de los controladores
        self.declare_parameter('controllers_ready', False)
        self.controllers_ready = self.get_parameter('controllers_ready').value
        
        # Velocidad lineal
        self.declare_parameter('linear_speed', 0.049)
        self.linear_speed = self.get_parameter('linear_speed').value # m/s
        self.speed_efect = self.linear_speed
        # Velocidad angular
        self.angular_speed = 0.0  # rad/s 
        self.yellow_speed = 0.0
        self.yellow_flag = False

        # Estado de identificación de color
        self.prev_color = 0
        self.new_color = 0
        self.color_state = 0

        self.error_linea = 0.0
        self.cont_cam = 0

        self.motion_state = 0

        self.yoloRec_msg = YoloAction()

        ## PUBLICADORES
        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Mensaje de velocidades para el Puzzlebot
        self.twist = Twist()
        # Publicador del valor de salida del PID
        self.pid_pub = self.create_publisher(Float32, 'pid_output', 10)
        self.pid_msg = Float32()
        
        ## SUSCRIPCIONES
        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.lineDetect_sub = self.create_subscription(Float32, 'line_error', self.lineDetector_callback, 10)
        # Suscriptor para el tópico /action (cominucación con identificaciones de YOLO)
        self.action_sub = self.create_subscription(YoloAction, 'action', self.action_callback, 10)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)
    
        # Timer de loop principal
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Autonomous Navigation Controller initialized!')
        
    def lineDetector_callback(self, msg):
        self.error_linea = msg.data

    def action_callback(self, msg):
        self.yoloRec_msg = msg

    def parameters_callback(self, params):
        for param in params:
            #system parameters check
            if param.name == "controllers_ready":
                #check if it is negative
                self.controllers_ready = param.value  # Update internal variable
                self.get_logger().info(f"controllers_ready updated to {self.controllers_ready}")
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
            elif param.name == "max_ang_vel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.max_ang_vel = param.value  # Update internal variable
                    self.get_logger().info(f"max_ang_vel updated to {self.max_ang_vel}")
            elif param.name == "min_ang_vel":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.min_ang_vel = param.value  # Update internal variable
                    self.get_logger().info(f"min_ang_vel updated to {self.min_ang_vel}")
            elif param.name == "init_time":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.init_time = param.value  # Update internal variable
                    self.get_logger().info(f"init_time updated to {self.init_time}")

        return SetParametersResult(successful=True)

    def control_loop(self):
        if self.controllers_ready:
            # Verificación de estado
            self.motion_state = self.decidir_accion()

            if self.motion_state == 0: # Estado de reposo
                
                if self.yoloRec_msg.rojo: # Si se detuvo por semáforo en rojo
                    self.get_logger().info('Estado de reposo por semaforo rojo')
                    self.reposo()
                    return
                elif self.yoloRec_msg.alto: # Si se detuvo por señal STOP
                    self.get_logger().info('Estado de reposo por STOP')
                    self.reposo()
                    return
                                
            elif self.motion_state == 1: # Estado de cruce (Lazo abierto)
                if not self.yoloRec_msg.verde: # Espera a ver semáforo en verde
                    self.get_logger().info('Estado de cruce, esperanco semaforo verde...')
                    self.reposo()
                    return
                else:
                    # Ejectua acción en función de la señal de movimiento
                    if self.yoloRec_msg.adelante:
                        self.seguirAdelante()
                    elif self.yoloRec_msg.girar_l:
                        self.girarIzquierda()
                    elif self.yoloRec_msg.girar_r:
                        self.girarDerecha()
                    
                    return 

            elif self.motion_state == 2: # Estado de seguimiento de linea con efectos              
                # Efectos
                if (self.yoloRec_msg.trabajo or self.yoloRec_msg.ceder):
                    self.get_logger().info('Estado de seguidor por senial triangular')
                    self.twist.linear.x = max(self.min_lin_vel, self.linear_speed * 0.9)

                elif self.yoloRec_msg.amarillo and not self.yoloRec_msg.rojo:
                    self.get_logger().info('Estado de seguidor por amarillo, esperando semaforo rojo...')    
                    self.speed_efect *= 0.99
                    self.twist.linear.x = max(0.0, self.speed_efect)
                else:
                    #self.get_logger().info('Estado de seguidor normal')
                    self.twist.linear.x = self.linear_speed
                    self.speed_efect = self.linear_speed
                    
                self.twist.angular.z = self.seguir_linea()
                self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

    def decidir_accion(self):
        if ((self.yoloRec_msg.rojo or self.yoloRec_msg.amarillo or self.yoloRec_msg.verde) and (self.yoloRec_msg.adelante or self.yoloRec_msg.girar_l or self.yoloRec_msg.girar_r)):
            motion_state = 1
        elif (self.yoloRec_msg.rojo or self.yoloRec_msg.alto):
            motion_state = 0
        else:
            motion_state = 2

        return motion_state
    
    def seguir_linea(self):
        pid_output = 0.0

        if abs(self.error_linea) > 0.0: 
            pid_output = self.pid_controller_angular_curv(self.error_linea)
            angular_velocity = self.saturate_with_deadband(pid_output, self.min_ang_vel, self.max_ang_vel)
            self.pid_msg.data = pid_output
            self.pid_pub.publish(self.pid_msg)
        else:
            angular_velocity = 0.0

        return angular_velocity

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
    
    def girarIzquierda(self):
        self.get_logger().info("Función de turn_left")
    
    def girarDerecha(self):
        self.get_logger().info("Función de turn_right")
    
    def seguirAdelante(self):
        self.get_logger().info("Función de ahead_only")
    
    def reposo(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        #self.get_logger().info("Función de reposo")
    
    def stop_handler(self, signum, frame):
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

# Main
def main(args=None):
    rclpy.init(args=args)
    node = trafficNavController()
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