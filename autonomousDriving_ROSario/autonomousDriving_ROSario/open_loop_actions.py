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
        self.declare_parameter('dist_factor', 1.0)    # segundos
        self.declare_parameter('ang_factor', 0.9)    # m/s
        self.declare_parameter('ready', False)
        self.declare_parameter('direction', 1)

        self.dist_factor = self.get_parameter('dist_factor').value
        self.ang_factor = self.get_parameter('ang_factor').value
        
        self.ready = self.get_parameter('ready').value
        self.direction = self.get_parameter('direction').value
        self.iter = False
        
        # Publicador para /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estado
        self.state = 0  # 0: forward, 1: rotate, 2: backward, 3: stop
        self.cont = 0

        self.state_start_time = 0.0

        # Velocidades
        self.linear_speed = 0.049 # m/s [0.025 a 0.37]
        self.angular_speed = 0.31 # rad/s [0.29 a 4.0]        

        self.dist_goal = 0.35
        self.ang_goal = np.deg2rad(90)

        # Tiempos de movimiento
        self.forward_time = 0.0   # Tiempo de movimiento lineal
        self.rotate_time = 0.0 # Tiempo de movimiento rotacional 
        
        # Timer para el ciclo de control
        self.timer_period = 1.0/15.0  # Frecuencia de muestreo: 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)
       

        self.get_logger().info('Open loop controller initialized!')
        
    def control_loop(self):
        if self.ready:
            if not self.iter:
                self.state_start_time = self.get_clock().now()
                self.forward_time = round(self.dist_goal * self.dist_factor, 2) / self.linear_speed   # Tiempo de movimiento lineal
                self.rotate_time =  round(self.ang_goal * self.ang_factor, 2) / abs(self.angular_speed) # Tiempo de movimiento rotacional
                self.angular_speed = abs(self.angular_speed) * self.direction
                self.iter = True

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
                    if self.cont == 0:
                        self.state = 1
                    else:
                        self.state = 3

                    self.state_start_time = now
                    self.get_logger().info('Finished moving forward. Starting rotation...')
            
            # Estado de movimiento rotacional
            elif self.state == 1:
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Rotating 90 degrees...')
                
                if elapsed_time >= self.rotate_time:
                    self.state = 0
                    self.cont += 1
                    self.state_start_time = now
                    self.get_logger().info('Finished rotation...')

            # Estado de reposo
            elif self.state == 3:
                self.state = 0
                self.cont = 0
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('Stopped.')
                self.ready = False
                self.iter = False

            # Publicación al tópico /cmd_vel
            self.cmd_vel_pub.publish(cmd)
        else:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)


    def parameters_callback(self, params):
        # Si el robot no está en movimiento, se actualizan los parámetros
        for param in params:
            # Actualización del parámetro 'time_goal'
            if param.name == 'dist_factor' and param.value > 0.0:
                self.dist_factor = param.value
                self.get_logger().info(f'dist_factor = {self.dist_factor}')
            elif param.name == 'ang_factor' and param.value > 0.0:
                self.ang_factor = param.value
                self.get_logger().info(f'ang_factor = {self.ang_factor}')
            elif param.name == 'ready' and param.value > 0.0:
                self.ready = param.value
                self.get_logger().info(f'ready = {self.ready}')
            elif param.name == 'direction':
                self.direction = param.value
                self.get_logger().info(f'direction = {self.direction}')
                
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
