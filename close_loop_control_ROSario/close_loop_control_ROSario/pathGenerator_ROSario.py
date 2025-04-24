#Imports
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from rosario_path.msg import RosarioPath

# Class Definition
class PathNode(Node):
    def __init__(self):
        super().__init__('path_ROSario')

        self.t = []

        #Declara los parametros que van en el parameter file
        self.declare_parameter('coordenadas_x', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) # self.declare_parameter('coordenadas_x', [])
        self.declare_parameter('coordenadas_y', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) # self.declare_parameter('coordenadas_y', [])
        

        self.declare_parameter('lin_vel', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) # self.declare_parameter('tiempo', [])
        
        self.declare_parameter('ang_vel', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) #self.declare_parameter('velocidad', [])
       
        self.declare_parameter('area', 0.0)

        self.x = self.get_parameter('coordenadas_x').get_parameter_value().double_array_value # self.x = self.get_parameter('coordenadas_x').value

        self.y = self.get_parameter('coordenadas_y').get_parameter_value().double_array_value # self.get_parameter('coordenadas_y').value
        
        # Cabiar estos parametros

        self.lin_vel = self.get_parameter('lin_vel').get_parameter_value().double_array_value # self.get_parameter('tiempo').value
        self.ang_vel = self.get_parameter('ang_vel').get_parameter_value().double_array_value # self.get_parameter('velocidad').value

        # lee arriba 

        self.a = self.get_parameter('area').value

        # Variables locales
        self.pwm = []
        self.time_exe = []

        self.publisher_ = self.create_publisher(RosarioPath, 'pose',10)

        self.index_publicacion = 0
        self.x_verified = []
        self.y_verified = []
        self.lin_verified =[]
        self.ang_verified = []

        self.calculate_path()

        self.timer = self.create_timer(1.0, self.publicar_mensaje)

    def calculate_path(self):
        """
        Verifica que los puntos dados esten dentro del area de trabajo
        Verifica que las velocidades sean correctas
        """

        self.get_logger().info(f" Iniciando Comprobacion")

        # Definicion del area de trabajo
        limite = self.a/2

        limite_min = -limite
        limite_max = limite

        # Velocidad Lineal
        v_lin_min = 0.025
        v_lin_max = 0.38

        # Velocidad Lineal
        v_ang_min = 0.29
        v_ang_max = 4.0


        # Temp variable para calculo de distancias
        coords=[(0.0,0.0)]

        # Se verifica si el vector esta vacio
        if len(self.x)==0:
            self.get_logger().info(f" No se a dado un vector de coordenadas")
            return

        # Se verifica que los vectores x y sean del mismo tamaño (Que no exista coordenadas incompletas)
        if len(self.x) == len(self.y):
            for i in range(len(self.x)):
                # Se lee que cada coordenada dada este dentro del area de trabajo
                if (self.x[i] >= limite_min and self.x[i] <= limite_max) and (self.y[i] >= limite_min and self.y[i] <= limite_max):
                    self.x_verified.append(self.x[i])
                    self.y_verified.append(self.y[i])
                    self.get_logger().info(f" Punto verificado {i}")
                    coords.append((self.x[i],self.y[i]))
                else:
                    self.get_logger().info(f" El punto dado {i} no esta dentro de los limites. ({self.x[i]}, {self.y[i]}), area: {self.a}")
                    return
        else:
            self.get_logger().info(f" El tamaño de vector 'x' no es el mismo que el vector 'y'")
            return
        
        if len(self.lin_vel) == 2:
            if (self.lin_vel[0] >= v_lin_min and self.lin_vel[0]<= v_lin_max):
                self.lin_verified.append(self.lin_vel[0])
                if (self.lin_vel[1] > self.lin_vel[0] and self.lin_vel[1] <= v_lin_max):
                    self.lin_verified.append(self.lin_vel[1])
                else:
                    self.get_logger().info(f" El valor lineal maximo no esta en rango")  
            else:
                self.get_logger().info(f" El valor lineal minimo no esta en rango")    
        else:
            self.get_logger().info(f" No se han introducido correctanente los limites lineales")
            return

        if len(self.ang_vel) == 2:
            if (self.ang_vel[0] >= v_ang_min and self.ang_vel[0]<= v_ang_max):
                self.ang_verified.append(self.ang_vel[0])
                if (self.ang_vel[1] > self.ang_vel[0] and self.ang_vel[1] <= v_ang_max):
                    self.ang_verified.append(self.ang_vel[1])
                else:
                    self.get_logger().info(f" El valor angular maximo no esta en rango")  
            else:
                self.get_logger().info(f" El valor angular minimo no esta en rango")    
        else:
            self.get_logger().info(f" No se han introducido correctanente los limites angulares")
            return

    def publicar_mensaje(self):
        # Contador de cuantos puntos se tienen que mandar al topcio
        if self.index_publicacion >= len(self.x_verified):
            self.get_logger().info("Todos los mensajes han sido publicados")
            self.timer.cancel() 
            return

        # Se instancia el custom msg
        msg = RosarioPath()

        # Se crea el objeto geometry_msg/Pose
        
        msg.path.position.x = self.x_verified[self.index_publicacion]
        msg.path.position.y = self.y_verified[self.index_publicacion]

        # Modificar esto

        msg.max_lin_vel = self.lin_verified[1]
        msg.min_lin_vel = self.lin_verified[0]
        msg.max_ang_vel = self.ang_verified[1]
        msg.min_ang_vel = self.ang_verified[0]

        # Modificar arriba

        # Se publica
        self.publisher_.publish(msg)
        self.get_logger().info(f" Publicado mensaje {self.index_publicacion}")
        self.index_publicacion += 1


def main(args=None):
    rclpy.init(args=args)
    nodo = PathNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
       
#Execute Node
if __name__ == '__main__':
    main()