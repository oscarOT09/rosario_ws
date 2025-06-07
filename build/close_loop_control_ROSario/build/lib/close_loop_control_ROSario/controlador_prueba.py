import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

class OpenLoopCtrl(Node):
    def __init__(self):
        super().__init__('open_loop_ctrl')

        # Posiciones objetivo
        self.X_ref = 0.0
        self.Y_ref = 0.0
        self.dist_ref = 0.0
        self.ang_ref = 0.0

        # Estados actuales
        self.X_robot = 0.0
        self.Y_robot = 0.0
        self.Th_robot = 0.0

        self.dist_robot = 0.0

        self.prev_X_robot = 0.0
        self.prev_Y_robot = 0.0


        # PID angular
        self.kp_ang = 0.25
        self.ki_ang = 0.005
        self.kd_ang = 0.002
        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # PID lineal
        self.kp_lin = 0.5
        self.ki_lin = 0.01
        self.kd_lin = 0.005
        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        # Velocidades
        self.twist = Twist()

        # Estados
        self.state = 0  # 0: rotación, 1: traslación, 2: detenerse
        self.robot_busy = False

        # Cola de objetivos
        self.path_queue = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]#self.path_queue = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        # Inicialización de posiciones
        self.prev_pose = np.array([0.0, 0.0])
        self.curr_pose = np.array([0.0, 0.0])
        self.next_pose = np.array([0.0, 0.0])
        
        self.prev_goal_X = 0.0
        self.prev_goal_Y = 0.0

        # Publicador y suscriptor
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometria, qos_profile_sensor_data)

        # Timer de control
        frecuencia_controlador = 200.0
        self.dt_pid = 1.0/200.0
        self.controller_timer = self.create_timer(1.0 / frecuencia_controlador, self.control_loop)

        self.get_logger().info('Open loop controller initialsized!')

    def control_loop(self):
        if not self.robot_busy and self.path_queue:
            self.prev_pose = self.curr_pose
            self.curr_pose = self.next_pose
            self.next_pose = self.path_queue.pop(0)
            
            self.X_ref, self.Y_ref = self.next_pose
            self.dist_ref = np.linalg.norm([self.X_ref - self.prev_goal_X, self.Y_ref - self.prev_goal_Y])
            self.dist_ref = self.dist_ref * 0.9

            #dx = self.X_ref - self.prev_goal_X
            #dy = self.Y_ref - self.prev_goal_Y
            self.ang_ref = self.delta_angle(self.prev_pose, self.curr_pose, self.next_pose) # np.arctan2(dy, dx)
            self.ang_ref = self.ang_ref * 0.8

            self.prev_goal_X = self.X_ref
            self.prev_goal_Y = self.Y_ref
            
            self.robot_busy = True
            self.state = 0
            self.get_logger().info(f'Nuevo objetivo: ({self.X_ref}, {self.Y_ref}) | {self.dist_ref} m | {np.rad2deg(self.ang_ref)}°')

        if self.robot_busy:
            if self.state == 0:
                
                #self.get_logger().info(f"Ang_robot: {np.rad2deg(self.Th_robot)} | Ang obj: {np.rad2deg(self.ang_ref)}")

                error = self.wrap_to_Pi(self.ang_ref - self.Th_robot)
                self.get_logger().info(f"Error angular: {self.ang_ref}-{self.Th_robot}={np.rad2deg(error):.2f}°")

                if abs(error) < np.deg2rad(10.0):
                    #self.get_logger().info("LLEGO A LIM. ERROR ANGULAR")
                    self.twist.angular.z = 0.0
                    
                    self.integral_ang = 0.0
                    self.prev_error_ang = 0.0

                    self.state = 1
                    #self.get_logger().info(f"Estado dese angular: {self.state}.")
                    self.get_logger().info("Rotación completada.")
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    
                    output = self.pid_controller_angular(error)
                    #self.get_logger().info(f"Salida PID ANGULAR: {output}")
                    self.twist.angular.z = self.saturate_with_deadband(output, 0.29, 3.0)
                    #self.get_logger().info(f"V.ANG: {self.twist.angular.z}")
                    #self.twist.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.twist)

            elif self.state == 1:
                # TRASLACIÓN
                self.dist_robot += np.linalg.norm([self.X_robot - self.prev_X_robot, self.Y_robot - self.prev_Y_robot])
                error = self.dist_ref - self.dist_robot
                self.get_logger().info(f"Error lineal: {self.dist_ref}-{self.dist_robot}={error:.3f} m")
                self.prev_X_robot = self.X_robot
                self.prev_Y_robot = self.Y_robot

                if error < 0.1:
                    #self.get_logger().info("LLEGO A LIM. ERROR LINEAL")
                    self.twist.linear.x = 0.0
                    self.state = 2
                    self.dist_robot = 0.0
                    self.integral_lin = 0.0
                    self.prev_error_lin = 0.0

                    #self.get_logger().info(f"Estado desde lineal: {self.state}.")
                    self.get_logger().info("Traslación completada.")
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    output = self.pid_controller_lineal(error)
                    #self.get_logger().info(f"Salida PID LINEAL: {output}")
                    self.twist.linear.x = self.saturate_with_deadband(output, 0.025, 0.38)
                    #self.get_logger().info(f"V.LIN: {self.twist.linear.x}")
                    #self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)

            elif self.state == 2:
                #self.twist = Twist()
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

                self.robot_busy = False
                self.get_logger().info("Objetivo alcanzado. Esperando siguiente...")
                self.cmd_vel_pub.publish(self.twist)

    def odometria(self, msg):
        self.X_robot = msg.pose.pose.position.x
        self.Y_robot = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quaternion)
        self.Th_robot = self.wrap_to_Pi(yaw)

    def delta_angle(self, prev, current, next_):
        prev = np.array(prev)
        current = np.array(current)
        next_ = np.array(next_)

        v1 = current - prev
        v2 = next_ - current

        angle1 = np.arctan2(v1[1], v1[0])
        angle2 = np.arctan2(v2[1], v2[0])

        delta = angle2 - angle1
        delta = (delta + np.pi) % (2 * np.pi) - np.pi  # Normalización del ágnulo entre -pi y pi
        return delta  # en radianes

    def wrap_to_Pi(self, angle):
        result = np.fmod((angle + np.pi), (2*np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

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
        if abs(output) < min_val:
            return min_val
        elif output > 0:
            return min(output,max_val)
        else:
            return max(output, -max_val)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()