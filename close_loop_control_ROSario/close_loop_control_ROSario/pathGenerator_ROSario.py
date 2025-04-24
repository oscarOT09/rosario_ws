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
        

        self.declare_parameter('tiempo', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) # self.declare_parameter('tiempo', [])
        
        self.declare_parameter('velocidad', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY) #self.declare_parameter('velocidad', [])
       
        self.declare_parameter('area', 0.0)

        self.x = self.get_parameter('coordenadas_x').get_parameter_value().double_array_value # self.x = self.get_parameter('coordenadas_x').value

        self.y = self.get_parameter('coordenadas_y').get_parameter_value().double_array_value # self.get_parameter('coordenadas_y').value
        
        self.t = self.get_parameter('tiempo').get_parameter_value().double_array_value # self.get_parameter('tiempo').value
        self.v = self.get_parameter('velocidad').get_parameter_value().double_array_value # self.get_parameter('velocidad').value

        self.a = self.get_parameter('area').value

        # Variables locales
        self.pwm = []
        self.time_exe = []

        self.publisher_ = self.create_publisher(RosarioPath, 'pose',10)

        self.index_publicacion = 0
        self.x_verified = []
        self.y_verified = []

        self.calculate_path()

        self.timer = self.create_timer(1.0, self.publicar_mensaje)

    def calculate_path(self):
        """
        Verifica que los puntos dados esten dentro del area de trabajo
        Calcula PWM y tiempo basado en parámetros de velocidad y tiempo
        """

        self.get_logger().info(f" Iniciando Comprobacion")

        # Definicion del area de trabajo
        limite = self.a/2

        limite_min = -limite
        limite_max = limite

        # Velocidad Lineal
        v_lin_min = 0.025
        v_lin_max = 0.38


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
        
        # Se ve si el array velocidad no este vacio
        if self.v[0] !=0:
            # Se compara que exista una velocidad para cada coordenada
            if len(self.x) == len(self.v):
                for j in range (len(self.v)):
                    # Revisa cada V para que este dentro del rango permitido
                    if (self.v[j] >= v_lin_min and self.v[j] <= v_lin_max):
                    #if v_lin_min <= self.v[j] <= v_lin_max: 
                        # Calcular Tiempo con base en su velocidad
                        p1 = coords[j]
                        p2 = coords[j+1]

                        distancia = np.linalg.norm(np.array(p2) - np.array(p1)) # distancia = np.linalg.norm(p2-p1)

                        cal_t = distancia / self.v[j] 

                        self.pwm.append(self.v[j])
                        self.time_exe.append(cal_t)
                        self.get_logger().info(f" Velocidad Verificada {j}")
                        self.get_logger().info(f" Tiempo calculado de {cal_t}")
                    else:
                        self.get_logger().info(f" La velocidad {j} = {self.v} sobrepasa lo permitido")
                        return
            else:
                self.get_logger().info(f" No se a dado un vector de velocidad del mismo tamaño que coordenadas")
        # Se ve si el array tiempo no este vacio
        elif self.t[0] != 0:
            # Se compara que exista un tiempo para cada coordenada
            if len(self.x) == len(self.t):
                for j in range (len(self.t)):
                    # Calcular Velocidad con base en Tiempo dado
                    p1 = coords[j]
                    p2 = coords[j+1]

                    distancia = np.linalg.norm(np.array(p2) - np.array(p1))

                    v_t = distancia / self.t[j]
                    if v_lin_min <= v_t <= v_lin_max: 
                        self.pwm.append(v_t)
                        self.time_exe.append(self.t[j])
                        self.get_logger().info(f" Tiempo Verificado {j}")
                        self.get_logger().info(f" Velocidad calculada de {v_t}")
                    else:
                        self.get_logger().info(f" El tiempo {j} = {self.t} va a {v_t} sobrepasa lo permitido")
                        return
            else:
                self.get_logger().info(f" No se a dado un vector de tiempo del mismo tamaño que coordenadas")
                return
        # Si ambos vectores Tiempo/Velocidad estan vacios 
        else:
            self.get_logger().info(f" No se a dado un vector de tiempo o de velocidad")
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

        msg.velocity = self.pwm[self.index_publicacion]
        msg.time = self.time_exe[self.index_publicacion]

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
