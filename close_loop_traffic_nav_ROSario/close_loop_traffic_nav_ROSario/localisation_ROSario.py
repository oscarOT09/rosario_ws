# Nodo de lozalización | Half-Term Challenge
  
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
        super().__init__('localisation_ROSario')

        #Set the parameters of the system
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Factores de conversión
        self.factor_x = 1.10087
        self.factor_y = 1.10806
        self.factor_th = 1.10088

        self._l = 0.18
        self._r = 0.05012
        
        self.rate = 200.0 #200.0
        self._sample_time = 1.0/self.rate
                    
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

        # Timer to update kinematics at ~200Hz
        self.timer = self.create_timer(1.0 / self.rate, self.run)


        self.get_logger().info("Odometry Node Started.")

    
    
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

            # Robot theta
            self.Th += (self.Omega * dt) * self.factor_th
            # Robot position in x
            self.X += (self.V * np.cos(self.Th) * dt) * self.factor_x
            # Robot position in y
            self.Y += (self.V * np.sin(self.Th) * dt) * self.factor_y

            self.last_time = current_time

            self.publish_odometry()


    def publish_odometry(self):
        """ Publishes odometry message with updated state """
        q1 = transforms3d.euler.euler2quat(0, 0, self.Th)
        
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = "base_footprint"
        
        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = q1[1]
        self.odom_msg.pose.pose.orientation.y = q1[2]
        self.odom_msg.pose.pose.orientation.z = q1[3]
        self.odom_msg.pose.pose.orientation.w = q1[0]
    
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