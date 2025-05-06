# Importaciones necesarias
import rclpy
from rclpy.node import Node
import numpy as np
#import transforms3d

#from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
#from rclpy.qos import qos_profile_sensor_data
#from rosario_path.msg import RosarioPath

# Definición de la clase
class colorIdentificator(Node):
    def __init__(self):
        super().__init__('color_identificator')
        
        # Muestreo
        frecuencia_controlador = 16.0

        self.state_start_time = self.get_clock().now()

        # Publicador para el tópico /cmd_vel (comunicación con el Puzzlebot)
        self.color_pub = self.create_publisher(Int32, 'color_id', 10)
        self.color_msg = Int32()

        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.main_loop)

        self.get_logger().info('Color Identificator initialized!')
    
    def main_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        if elapsed >= 0 and elapsed <= 3:
            self.color_msg.data = 1
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')
        elif elapsed >= 3 and elapsed <= 6:
            self.color_msg.data = 3
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')
        elif elapsed >= 6 and elapsed <= 9:
            self.color_msg.data = 0
            self.color_pub.publish(self.color_msg)
            self.get_logger().info(f'Color Message: {self.color_msg.data} at {elapsed} s')


# Main
def main(args=None):
    rclpy.init(args=args)
    node = colorIdentificator()
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