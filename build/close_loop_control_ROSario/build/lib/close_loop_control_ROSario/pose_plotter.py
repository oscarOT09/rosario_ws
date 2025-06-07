import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from rclpy import qos

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        self.x_data = []
        self.y_data = []
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            qos.qos_profile_sensor_data)
        
        self.get_logger().info("PosePlotter node started. Listening to /odom...")

    def listener_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def plot_trajectory(self):
        plt.figure()
        plt.plot(self.x_data, self.y_data, marker='o', linestyle='-')
        plt.title("Trayectoria estimada del robot")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = PosePlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info('Interrupt received. Plotting trajectory...')
        plotter.plot_trajectory()
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
