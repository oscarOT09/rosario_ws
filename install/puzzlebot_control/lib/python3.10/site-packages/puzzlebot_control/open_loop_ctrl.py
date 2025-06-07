# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        # Parámetros
        self.declare_parameter('time_goal', 0.0)    # segundos
        self.declare_parameter('velocity_goal', 0.0)    # m/s

        # Bandera para identificación de movimiento del robot
        self.robot_busy = False

        # Distancia de las aristas de la figura       
        self.side_size = 2.0

        # Contador de lados
        self.lados = 0

        # Número de aristas de la figura
        self.side_num = 4

        # Ángulo de rotación
        self.degrees_rot = 90

        # Publicador para /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estado
        self.state = 0  # 0: forward, 1: rotate, 2: backward, 3: stop
    
        # Velocidades
        self.linear_speed = self.get_parameter('velocity_goal').value # m/s [0.025 a 0.38]
        self.angular_speed = 1.0 # rad/s [0.29 a 4.0]        

        # Tiempos de movimiento
        self.forward_time = self.get_parameter('time_goal').value   # Tiempo de movimiento lineal
        self.rotate_time =  np.deg2rad(round(self.degrees_rot * 0.89444, 2)) / self.angular_speed # Tiempo de movimiento rotacional 
        
        # Timer para el ciclo de control
        self.timer_period = 0.1  # Frecuencia de muestreo: 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Open loop controller initialized!')
        
    def control_loop(self):
        # Si el robot está en movimiento, se trabaja entre los estados
        if self.robot_busy:
            now = self.get_clock().now()
            elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

            self.get_logger().info(f"Start: {self.state_start_time.nanoseconds * 1e-9}, NOW: {now.nanoseconds * 1e-9:.2f}s")
            self.get_logger().info(f"State: {self.state}, Elapsed: {elapsed_time:.2f}s")

            # Creación del mensaje Geometry Twist
            cmd = Twist()

            # Estado de movimiento lineal
            if self.state == 0:
                cmd.linear.x = self.linear_speed
                self.get_logger().info('Moving forward...')
                
                if elapsed_time >= self.forward_time:
                    self.state = 1
                    self.state_start_time = now
                    self.get_logger().info('Finished moving forward. Starting rotation...')
            
            # Estado de movimiento rotacional
            elif self.state == 1:
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Rotating 90 degrees...')
                
                if elapsed_time >= self.rotate_time:
                    self.lados += 1 # Se incrementa el contador de lados completados
                    if self.lados < self.side_num: # Si aún no se completa el numero de aristas totales, se regresa al estado 0
                        self.state = 0
                        
                    else: # Si ya terminó el número de aristas de la figura, va al estado 3
                        self.state = 3
                        self.state_start_time = now
                        self.get_logger().info('Finished rotation. Stopping...')
                    
                    self.state_start_time = now
                    self.get_logger().info('Finished rotation. Moving forward...')

            # Estado de reposo
            elif self.state == 3:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                self.robot_busy = False
                self.linear_speed = 0.0
                self.forward_time = 0.0

                self.get_logger().info('Stopped.')

            # Publicación al tópico /cmd_vel
            self.cmd_vel_pub.publish(cmd)

    def parameters_callback(self, params):
        # Si el robot está en movimiento, no se permiten cambios en los parámetros
        if self.robot_busy:
            self.get_logger().warn("Robot en movimiento, no se permiten cambios de parámetros.")
            return SetParametersResult(successful=False)

        # Si el robot no está en movimiento, se actualizan los parámetros
        for param in params:
            # Actualización del parámetro 'time_goal'
            if param.name == 'time_goal' and param.value > 0.0:
                # Se verifica que el valor esté dentro del rango
                if param.value >= 5.30 and param.value <= 80.0:
                    self.linear_speed = self.side_size / param.value
                    self.forward_time = param.value
                    self.get_logger().info(f'Se usó time_goal = {param.value} s')
                else:
                    self.get_logger().warn("time_goal debe estar en el rango [5.30, 80.0]")
            
            # Actualización del parámetro 'velocity_goal'
            elif param.name == 'velocity_goal' and param.value > 0.0:
                # Se verifica que el valor esté dentro del rango
                if param.value >= 0.025 and param.value <= 0.38:
                    self.linear_speed = param.value
                    self.forward_time = self.side_size / self.linear_speed
                    self.get_logger().info(f'Se usó velocity_goal = {param.value} m/s')
                else:
                    self.get_logger().warn("velocity_goal debe estar en el rango [0.025, 0.38]")

        # Si los valores ingresados son diferentes de 0.0, se inicia el movimiento del robot
        if self.forward_time > 0.0 or self.linear_speed > 0.0:
            self.state = 0
            self.lados = 0
            self.robot_busy = True
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f'Auto-tune: Velocidad = {self.linear_speed} m/s, Tiempo por lado = {self.forward_time} s')

        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()
