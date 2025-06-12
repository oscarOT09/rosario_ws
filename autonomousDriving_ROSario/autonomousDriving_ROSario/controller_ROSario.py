# Nodo de control |  Final-Term Challenge
# Equipo ROSario

# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
import signal

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from action_msg.msg import YoloAction

# Definición de la clase
class trafficNavController(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Muestreo
        frecuencia_controlador = 10.0

        self.declare_parameter('max_ang_vel', 0.8)
        self.declare_parameter('min_ang_vel', 0.25)

        # Parámetros del robot
        self.max_lin_vel = 0.37
        self.min_lin_vel = 0.025
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.min_ang_vel = self.get_parameter('min_ang_vel').value

        # Delta tiempo controlador
        self.dt_pid = 1.0/frecuencia_controlador

        # PID
        self.declare_parameter('kp_ang_curv', 0.55)
        self.declare_parameter('ki_ang_curv', 0.0)
        self.declare_parameter('kd_ang_curv', 0.01)

        self.kp_ang_curv = self.get_parameter('kp_ang_curv').value # 1.2
        self.ki_ang_curv = self.get_parameter('ki_ang_curv').value # 0.5
        self.kd_ang_curv = self.get_parameter('kd_ang_curv').value # 0.5
        self.integral_ang_curv = 0.0
        self.prev_error_ang_curv = 0.0

        # Bandera para actualización de los parámetros de los controladores
        self.declare_parameter('controllers_ready', False)
        self.controllers_ready = self.get_parameter('controllers_ready').value
        
        # Velocidad lineal
        self.declare_parameter('linear_speed', 0.05)
        self.linear_speed = self.get_parameter('linear_speed').value # m/s
        self.speed_efect = self.linear_speed

        self.declare_parameter('ahead_lin_por', 1.14)
        self.ahead_lin_por = self.get_parameter('ahead_lin_por').value

        self.declare_parameter('corr_dir', 0.0)
        self.corr_dir = self.get_parameter('corr_dir').value

        self.declare_parameter('yellow_decrement', 0.01)
        self.yellow_decrement = self.get_parameter('yellow_decrement').value

        self.declare_parameter('num_cont_stop', 15)
        self.num_cont_stop = self.get_parameter('num_cont_stop').value

        self.linear_speed_msg = 0.0
        self.angular_speed_msg = 0.0

        #### OPEN LOOP ####
        self.angular_speed_open = self.max_lin_vel
        self.linear_speed_open = self.linear_speed
        
        self.iter_rot = False
        self.setup_turn = False
        
        self.openLoop_state = 0
        self.openLoop_cont = 0

        self.dist_goal = 0.35
        self.ang_goal = np.deg2rad(90)

        self.forward_time = 0.0   # Tiempo de movimiento lineal
        self.rotate_time = 0.0 # Tiempo de movimiento rotacional

        self.state_start_time = 0.0
        self.turn_direction = 0

        self.setup_ahead = False
        self.iter_ahead = False
        
        self.paso_flag = False
        self.yellow_speed = 0.0

        self.stopSIG = False
        self.stopTL = False

        self.motion_state = 0

        ############33
        self.trafficLight_state = 0
        self.direction_state = 0
        self.linear_speed_msg = 0.0
        self.angular_speed_msg = 0.0
        self.stop_flag = False
        self.reduce_zone = False
        self.stop_cont = False
        self.cont_stop = 0
        #############33
        ## PUBLICADORES

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        
        # Publicador del valor de salida del PID
        self.pid_pub = self.create_publisher(Float32, 'pid_output', 10)
        self.pid_msg = Float32()
        
        ## SUSCRIPCIONES
        
        # Suscriptor para el tópico /pose (comunicación con el nodo Path Generator)
        self.lineDetect_sub = self.create_subscription(Float32, 'line_error', self.lineDetector_callback, 10)
        self.error_linea = 0.0

        # Suscriptor para el tópico /action (cominucación con identificaciones de YOLO)
        self.action_sub = self.create_subscription(YoloAction, 'action', self.action_callback, 10)
        self.yoloRec_msg = YoloAction()

        ## Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)
    
        ## Timer de loop principal
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Autonomous Navigation Controller initialized!')
        
    def lineDetector_callback(self, msg):
        self.error_linea = msg.data

    def action_callback(self, msg):
        self.yoloRec_msg = msg

    def parameters_callback(self, params):
        for param in params:
            # Ganancias del sistema
            if param.name == "controllers_ready":
                self.controllers_ready = param.value  # Actualizar variable interna
                self.get_logger().info(f"controllers_ready updated to {self.controllers_ready}")
            elif param.name == "kp_ang_curv":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp_ang_curv = param.value  # Actualizar variable interna
                    self.get_logger().info(f"kp_ang_curv updated to {self.kp_ang_curv}")
            elif param.name == "ki_ang_curv":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki_ang_curv = param.value  # Actualizar variable interna
                    self.get_logger().info(f"ki_ang_curv updated to {self.ki_ang_curv}")
            elif param.name == "kd_ang_curv":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd_ang_curv = param.value  # Actualizar variable interna
                    self.get_logger().info(f"kd_ang_curv updated to {self.kd_ang_curv}")
            elif param.name == "linear_speed":
                if (param.value < 0.025 and param.value > 0.37):
                    self.get_logger().warn("Invalid linear_speed! It's out of range.")
                    return SetParametersResult(successful=False, reason="linear_speed cannot be negative")
                else:
                    self.linear_speed = param.value  # Actualizar variable interna
                    self.get_logger().info(f"linear_speed updated to {self.linear_speed}")
            elif param.name == "yellow_decrement":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid yellow_decrement! It's out of range.")
                    return SetParametersResult(successful=False, reason="yellow_decrement cannot be negative")
                else:
                    self.yellow_decrement = param.value  # Actualizar variable interna
                    self.get_logger().info(f"yellow_decrement updated to {self.yellow_decrement}")
            elif param.name == "num_cont_stop":
                if (param.value < 0):
                    self.get_logger().warn("Invalid num_cont_stop! It's out of range.")
                    return SetParametersResult(successful=False, reason="num_cont_stop cannot be negative")
                else:
                    self.num_cont_stop = param.value  # Actualizar variable interna
                    self.get_logger().info(f"num_cont_stop updated to {self.num_cont_stop}")
            elif param.name == "ahead_lin_por":
                if (param.value < 0):
                    self.get_logger().warn("Invalid ahead_lin_por! It's out of range.")
                    return SetParametersResult(successful=False, reason="ahead_lin_por cannot be negative")
                else:
                    self.ahead_lin_por = param.value  # Actualizar variable interna
                    self.get_logger().info(f"ahead_lin_por updated to {self.ahead_lin_por}")
            elif param.name == "corr_dir":
                self.corr_dir = param.value  # Actualizar variable interna
                self.get_logger().info(f"corr_dir updated to {self.corr_dir}")
            elif param.name == "max_ang_vel":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid max_ang_vel! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="max_ang_vel cannot be negative")
                else:
                    self.max_ang_vel = param.value  # Actualizar variable interna
                    self.get_logger().info(f"max_ang_vel updated to {self.max_ang_vel}")
            elif param.name == "min_ang_vel":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid min_ang_vel! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="min_ang_vel cannot be negative")
                else:
                    self.min_ang_vel = param.value  # Actualizar variable interna
                    self.get_logger().info(f"min_ang_vel updated to {self.min_ang_vel}")
        return SetParametersResult(successful=True)

    def control_loop(self):
        if self.controllers_ready:

            if self.stop_cont:
                if self.cont_stop < self.num_cont_stop:
                    self.cont_stop += 1
                    return
                else:
                    self.cont_stop = 0
                    self.stop_cont = False

            if self.iter_rot:
                self.girar(self.turn_direction)
                return
            if self.iter_ahead:
                self.seguirAdelante()
                return
            self.get_logger().info(f'[MSG] alto: {self.yoloRec_msg.alto}, rojo: {self.yoloRec_msg.rojo}, amarillo: {self.yoloRec_msg.amarillo}, verde: {self.yoloRec_msg.verde}, adelante: {self.yoloRec_msg.adelante}, girar_l: {self.yoloRec_msg.girar_l}, girar_r: {self.yoloRec_msg.girar_r}, trabajo: {self.yoloRec_msg.trabajo}, ceder: {self.yoloRec_msg.ceder}')
            # Decision de maquina de estados
            self.decidir_accion()

            if self.trafficLight_state == 1:
                self.linear_speed_msg = 0.0
                self.angular_speed_msg = 0.0
                self.yellow_speed = self.linear_speed

            elif self.trafficLight_state == 2:
                self.yellow_speed *= (1.0-self.yellow_decrement)
                self.linear_speed_msg = max(self.min_lin_vel, self.yellow_speed)
                self.angular_speed_msg = self.seguir_linea()
            
            elif self.trafficLight_state == 3:
                if self.direction_state == 1:
                    self.turn_direction = 1
                    self.girar(self.turn_direction)
                    return
            
                elif self.direction_state == 2:
                    self.turn_direction = -1
                    self.girar(self.turn_direction)
                    return
            
                elif self.direction_state == 3:
                    self.seguirAdelante()
                    return
                else:
                    self.linear_speed_msg = self.linear_speed
                    self.angular_speed_msg = self.seguir_linea()
            
            else:
                if self.stop_flag:
                    self.linear_speed_msg = 0.0
                    self.angular_speed_msg = 0.0
                    self.stop_cont = True
                else:
                    if self.reduce_zone == True:
                        self.linear_speed_msg = self.linear_speed*0.5
            
                    else:
                        self.linear_speed_msg = self.linear_speed
            
                    self.angular_speed_msg = self.seguir_linea()

            self.twist.linear.x = self.linear_speed_msg
            self.twist.angular.z = self.angular_speed_msg
            self.cmd_vel_pub.publish(self.twist)

        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

    def decidir_accion(self):
        # Revisa señal de giro
        if self.yoloRec_msg.girar_l:
            self.direction_state = 1
        elif self.yoloRec_msg.girar_r:
            self.direction_state = 2
        elif self.yoloRec_msg.adelante:
            self.direction_state = 3
            

        # Maquina de estados de semaforo
        
        if self.yoloRec_msg.verde:
            self.trafficLight_state = 3
        elif self.yoloRec_msg.amarillo:
            self.trafficLight_state = 2
        elif self.yoloRec_msg.rojo:
            self.trafficLight_state = 1
        else:
            self.trafficLight_state = 0

        if self.yoloRec_msg.alto:
            self.stop_flag = True
        else:
            self.stop_flag = False


        if self.yoloRec_msg.trabajo or self.yoloRec_msg.ceder:
            self.reduce_zone = True
        else:
            self.reduce_zone = False
    
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
    
    def girar(self, direction):
        if not self.setup_turn:     
            if direction > 0:
                    self.get_logger().info("Función de turn_left")
            else:
                self.get_logger().info("Función de turn_right")
            
            self.state_start_time = self.get_clock().now()
            self.forward_time = round(self.dist_goal * self.ahead_lin_por, 2) / self.linear_speed_open   # Tiempo de movimiento lineal
            self.rotate_time =  round(self.ang_goal * 0.86, 2) / abs(self.angular_speed_open) # Tiempo de movimiento rotacional
            self.angular_speed_open = abs(self.angular_speed_open) * direction
            self.setup_turn = True
            self.iter_rot = True
        
        
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        
        # Estado de movimiento lineal
        if self.openLoop_state == 0:
            self.twist.linear.x = self.linear_speed_open
            self.twist.angular.z = 0.0
            #self.get_logger().info('Moving forward...')
            
            if elapsed_time >= self.forward_time:
                if self.openLoop_cont == 0:
                    self.openLoop_state = 1
                else:
                    self.openLoop_state = 3

                self.state_start_time = self.get_clock().now() #self.state_start_time = now
                #self.get_logger().info('Finished moving forward. Starting rotation...')
        
        # Estado de movimiento rotacional
        elif self.openLoop_state == 1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed_open
            #self.get_logger().info('Rotating 90 degrees...')
            
            if elapsed_time >= self.rotate_time:
                self.openLoop_state = 0
                self.openLoop_cont += 1
                self.forward_time = round(0.05 * 0.9, 2) / self.linear_speed_open
                self.state_start_time = self.get_clock().now() #self.state_start_time = now
                #self.get_logger().info('Finished rotation...')

        # Estado de reposo
        elif self.openLoop_state == 3:
            self.openLoop_state = 0
            self.openLoop_cont = 0
            self.direction_state = 0
            self.iter_rot = False
            self.setup_turn = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info('Movimiento finalizado.')
        
        self.cmd_vel_pub.publish(self.twist)
        
    
    def seguirAdelante(self):
        if not self.setup_ahead:     
            self.get_logger().info("Función de ahead_only")
            self.state_start_time = self.get_clock().now()
            self.forward_time = round(self.dist_goal * 1.1, 2) / self.linear_speed_open   # Tiempo de movimiento lineal
            self.setup_ahead = True
            self.iter_ahead = True
        
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        
        # Estado de movimiento lineal
        if self.openLoop_state == 0:
            self.twist.linear.x = self.linear_speed_open
            self.twist.angular.z = self.corr_dir*0.021
            #self.get_logger().info('Moving forward...')
            
            if elapsed_time >= self.forward_time:
                self.openLoop_state = 3
                self.state_start_time = self.get_clock().now() #self.state_start_time = now
                #self.get_logger().info('Finished moving forward. Starting rotation...')
        
        # Estado de reposo
        elif self.openLoop_state == 3:
            self.openLoop_state = 0
            self.iter_ahead = False
            self.setup_ahead = False
            self.direction_state = 0
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info('Movimiento finalizado.')
        
        self.cmd_vel_pub.publish(self.twist)
    
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