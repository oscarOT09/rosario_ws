# Nodo graficador de pose del robot | Half-Term Challenge

import rclpy
import matplotlib.pyplot as plt
import numpy as np
import signal

from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from rosario_path.msg import RosarioPath



class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')

        self.x_data = []
        self.y_data = []
        self.colors = []

        # Color_ID:
        # 0 - Azul (sin semáforo)
        # 1 - Rojo
        # 2 - Amarillo
        # 3 - Verde
        self.color_map = {
            0: 'blue',
            1: 'red',
            2: 'yellow',
            3: 'green'
        }
        self.current_color_ID = 0

        self.path_queue = []

        # Configuración del gráfico
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Trayectoria estimada del robot")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.axis("equal")
        self.ax.grid(True)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            qos.qos_profile_sensor_data)

        self.path_subscription = self.create_subscription(
            RosarioPath,
            '/pose',
            self.pose_callback,
            10
        )
        self.subscription_colorsID = self.create_subscription(
            Int32,
            '/color_id',
            self.colors_callback,
            10
        )

        self.timer = self.create_timer(1.0/250.0, self.update_plot)

        self.get_logger().info("PosePlotter node started. Listening to /odom and /color_ID...")

    def odometry_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)
        self.colors.append(self.color_map.get(self.current_color_ID, 'blue'))  # color por default: azul

    def pose_callback(self, msg):
        # Descomposición para almacenamiento local en variables 
        new_point = np.array([msg.path.position.x, msg.path.position.y])

        # Guardado en queue
        self.path_queue.append((new_point))

    def colors_callback(self, msg):
        self.current_color_ID = msg.data

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Trayectoria estimada del robot")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.axis("equal")
        self.ax.grid(True)

        for point in self.path_queue:
            self.ax.plot(point[0], point[1], marker='o', color='black')

        for x, y, c in zip(self.x_data, self.y_data, self.colors):
            self.ax.plot(x, y, marker='o', color=c)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    plotter = PosePlotter()
    signal.signal(signal.SIGINT, plotter.stop_handler)
    
    try:
        rclpy.spin(plotter)
    except SystemExit:
        plotter.get_logger().info('Nodo finalizado limpiamente.')
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()