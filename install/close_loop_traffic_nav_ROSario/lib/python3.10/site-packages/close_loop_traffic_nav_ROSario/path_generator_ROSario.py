#Nodo Path Generator | Half-Term Challenge

#Imports
import numpy as np
import rclpy
import signal

from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from rosario_path.msg import RosarioPath

# Class Definition
class PathNode(Node):
    def __init__(self):
        super().__init__('path_ROSario')

        # Declararión local de parámetros
        self.declare_parameter('coordenadas_x', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('coordenadas_y', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('lin_vel', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('ang_vel', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('origen_robot', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('area_largo', 0.0)
        self.declare_parameter('area_ancho', 0.0)

        # Lectura de parámetros del parameter file
        self.x = self.get_parameter('coordenadas_x').get_parameter_value().double_array_value
        self.y = self.get_parameter('coordenadas_y').get_parameter_value().double_array_value
        self.lin_vel = self.get_parameter('lin_vel').get_parameter_value().double_array_value
        self.ang_vel = self.get_parameter('ang_vel').get_parameter_value().double_array_value
        self.origen_robot = self.get_parameter('origen_robot').get_parameter_value().double_array_value
        self.area_largo = self.get_parameter('area_largo').value
        self.area_ancho = self.get_parameter('area_ancho').value

        self.msg = RosarioPath()
        self.goals_publisher = self.create_publisher(RosarioPath, 'pose', 10)

        self.index_publicacion = 0
        self.x_verified = []
        self.y_verified = []
        self.lin_verified =[]
        self.ang_verified = []

        self.path_verified = False

        self.calculate_path()

        self.timer = self.create_timer(1.0, self.publicar_mensaje)

    def calculate_path(self):
        """
        Verifica que los puntos dados esten dentro del area de trabajo
        Verifica que las velocidades sean correctas
        """

        self.get_logger().info(f"Iniciando Comprobacion")

        # Límites del área de trabajo
        xmin_lim = -self.area_largo/2
        xmax_lim = self.area_largo/2
        
        ymin_lim = -self.area_ancho/2
        ymax_lim = self.area_ancho/2

        # Límites de velocidad Lineal
        v_lin_min = 0.025
        v_lin_max = 0.38

        # Límites de velocidad angular
        v_ang_min = 0.29
        v_ang_max = 4.0

        # Matriz de transformación homogenea con el origen del robot respecto al mundo
        mat_trans =np.array([
                            [1.0, 0.0, 0.0, self.origen_robot[0]],
                            [0.0, 1.0, 0.0, self.origen_robot[1]],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]
                            ])
        
        # Se verifica si el vector esta vacio
        if len(self.x)==0 or len(self.y)==0:
            self.get_logger().info(f"No se han declarado vectores de coordenadas completos")
            return

        # Verificación de las coordenadas de navegación
        if len(self.x) == len(self.y): # Se verifica que los vectores X y Y sean del mismo tamaño (Que no exista coordenadas incompletas)
            for i in range(len(self.x)): 
                tmp_coor = np.array([[self.x[i]],[self.y[i]],[0],[1]])
                coor_trans = mat_trans @ tmp_coor
                # Se lee que cada coordenada dada este dentro del area de trabajo
                if (coor_trans[0][0] >= xmin_lim and coor_trans[0][0] <= xmax_lim) and (coor_trans[1][0] >= ymin_lim and coor_trans[1][0] <= ymax_lim):
                    self.x_verified.append(self.x[i])
                    self.y_verified.append(self.y[i])
                    self.get_logger().info(f"El punto ({self.x[i]}, {self.y[i]}) transformado a ({coor_trans[0][0]}, {coor_trans[1][0]}) se encuentra en el rango del área de trabajo")
                else:
                    self.get_logger().info(f"El punto ({self.x[i]}, {self.y[i]}) transformado a ({coor_trans[0][0]}, {coor_trans[1][0]}) no esta dentro de los limites del área de trabajo (x=[{xmin_lim}, {xmax_lim}], y=[{ymin_lim}, {ymax_lim}])")
                    return
        else:
            self.get_logger().info(f"El tamaño de los vectores de coordenadas X y Y no coinciden, hay coordenadas incompletas.")
            return
        
        # Verificación de los límites de la velocidad lineal
        if len(self.lin_vel) == 2:
            if (self.lin_vel[0] >= v_lin_min and self.lin_vel[0]<= v_lin_max):
                self.lin_verified.append(self.lin_vel[0])  
            else:
                self.get_logger().info(f"El valor lineal minimo no esta dentro del rango")
                return
            
            if (self.lin_vel[1] > self.lin_vel[0] and self.lin_vel[1] <= v_lin_max):
                self.lin_verified.append(self.lin_vel[1])
            else:
                self.get_logger().info(f"El valor lineal maximo no esta dentro del rango")
                return    
        else:
            self.get_logger().info(f"Los límites de la velocidad lineal están incompletos")
            return

        # Verificación de los límites de la velocidad angular
        if len(self.ang_vel) == 2:
            if (self.ang_vel[0] >= v_ang_min and self.ang_vel[0]<= v_ang_max):
                self.ang_verified.append(self.ang_vel[0])  
            else:
                self.get_logger().info(f" El valor angular minimo no esta en rango")
                return

            if (self.ang_vel[1] > self.ang_vel[0] and self.ang_vel[1] <= v_ang_max):
                self.ang_verified.append(self.ang_vel[1])
            else:
                self.get_logger().info(f" El valor angular maximo no esta en rango")
                return    
        else:
            self.get_logger().info(f"Los límites de la velocidad angular están incompletos")
            return
        
        self.path_verified = True

    def publicar_mensaje(self):
        if self.path_verified:
            # Control de publicación en base al número de coordenadas
            if self.index_publicacion >= len(self.x_verified):
                self.get_logger().info("Todos los mensajes han sido publicados")
                self.timer.cancel() 
                return

            # Declaración de los datos para cada parte del mensaje personalizado
            self.msg.path.position.x = self.x_verified[self.index_publicacion]
            self.msg.path.position.y = self.y_verified[self.index_publicacion]
            self.msg.max_lin_vel = self.lin_verified[1]
            self.msg.min_lin_vel = self.lin_verified[0]
            self.msg.max_ang_vel = self.ang_verified[1]
            self.msg.min_ang_vel = self.ang_verified[0]

            # Publicación del mensaje personalizado
            self.goals_publisher.publish(self.msg)
            self.get_logger().info(f" Publicado mensaje {self.index_publicacion}")
            self.index_publicacion += 1
        else:
            self.get_logger().info(f"Errores en el parameter file (/config/params.yaml)")
            self.timer.cancel()

    def stop_handler(self, signum, frame):
        '''Manejo de interrupción por teclado (ctrl + c)'''
        self.get_logger().info("Deteniendo nodo por interrupción por teclado...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    nodo = PathNode()
    signal.signal(signal.SIGINT, nodo.stop_handler)
    
    try:
        rclpy.spin(nodo)
    except SystemExit:
        nodo.get_logger().info('Nodo finalizado limpiamente.')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
    
#Execute Node
if __name__ == '__main__':
    main()