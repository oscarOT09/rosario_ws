# Actividad R_2. Cálculo del error de un robot móvil diferencial
# Equipo ROSario

'''
2. Crear un nodo para publicar el cálculo de los errores 𝑒𝑑 y 𝑒𝜃 del robot.
- Establezca un objetivo, y conduzca el robot alrededor, comprobando que el ángulo a el objetivo y la distancia desde el objetivo se actualizan correctamente.
- Recuerde que todos los ángulos deben estar dentro de un círculo.
'''

import rclpy
import transforms3d
import numpy as np
import signal

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from rcl_interfaces.msg import SetParametersResult

class DeadReckoning(Node):

    def __init__(self):
        super().__init__('errors_pub')

        #Parametros 
        self.declare_parameter('distancia_obj', 0.0)
        self.declare_parameter('theta_obj', 0.0)

        self.distancia_obj = self.get_parameter('distancia_obj').value
        self.theta_obj = self.get_parameter('theta_obj').value

        self.distance_robot = 0.0

        #Set the parameters of the system
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0
        self._l = 0.18
        self._r = 0.05
        self._sample_time = 0.01
        self.rate = 200.0
                    
        # Internal state
        self.first = True
        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0

        #Variables to be used
        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0

        #Messages to be used
        self.wr = Float32()
        self.wl = Float32()
        self.odom_msg = Odometry()

        # Subscriptions
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)
        self.e_dist_pub = self.create_publisher(Float32, 'e_dist', 10)
        self.e_theta_pub = self.create_publisher(Float32, 'e_theta', 10)

        # Timer to update kinematics at ~100Hz
        self.timer = self.create_timer(1.0 / self.rate, self.run)  # 100 Hz

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Errors Publisher Node Started.")

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "distancia_obj":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid goal distance! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="Goal distance cannot be negative")
                else:
                    self.distancia_obj = param.value  # Update internal variable
                    self.get_logger().info(f"Goal distance updated to {self.distancia_obj}")
            elif param.name == "theta_obj":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid goal angle! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="Goal angle cannot be negative")
                elif (abs(param.value) > 360):
                    self.get_logger().warn("Invalid goal angle! It has to be in the range [0, 2*pi].")
                    return SetParametersResult(successful=False, reason="Goal distance cannot out of a circle range")
                else:
                    self.theta_obj = param.value  # Update internal variable
                    self.get_logger().info(f"Goal theta updated to {self.theta_obj}")
        return SetParametersResult(successful=True)
    
    # Callbacks
    def encR_callback(self, msg):
        self.wr = msg

    def encL_callback(self, msg):
        self.wl = msg

    def run(self):

        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.current_time = self.start_time
            self.first = False
            return
        
        """ Updates robot position based on real elapsed time """
        # Get current time and compute dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        
        if dt > self._sample_time:

            #Wheel Tangential Velocities
            self.v_r = self._r  * self.wr.data
            self.v_l = self._r  * self.wl.data

            #Robot Velocities
            self.V = (1/2.0) * (self.v_r + self.v_l)
            self.Omega = (1.0/self._l) * (self.v_r - self.v_l)

            # Robot position in x
            self.X += self.V * np.cos(self.Th) * dt
            # Robot position in y
            self.Y += self.V * np.sin(self.Th) * dt
            # Robot theta
            if (abs(self.Th) > 360):
                self.Th = 0
            else: 
                self.Th += self.Omega * dt

            #Distancia 
            self.distance_robot += abs(self.V) * dt

            # self.cont += 1

            self.last_time = current_time

            self.publish_odometry()
            self.publish_errors()

    def publish_errors(self):
        """Publishes errors message with updated state """
        error_dist = Float32()
        error_dist.data = self.distancia_obj - self.distance_robot

        error_theta = Float32()
        error_theta.data = np.rad2deg(np.deg2rad(self.theta_obj) - self.Th)

        self.e_dist_pub.publish(error_dist)
        self.e_theta_pub.publish(error_theta)

    def publish_odometry(self):
        """ Publishes odometry message with updated state """

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = "base_footprint"
        
        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.z = self.Th
    
        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(self.odom_msg)



    def stop_handler(self,signum, frame):
        """Handles Ctrl+C (SIGINT)."""
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit



def main(args=None):

    rclpy.init(args=args)

    node = DeadReckoning()

    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()