# Lanzador de Yolo | Final-Term Challenge
# Equipo ROSario

# Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
 
    yolo_recognition = Node(
                            name="yoloPred_node",
                            package="autonomousDriving_ROSario",
                            executable="yolov8_recognition",
                            output = 'screen'
                            )
    
    yolo_bridge = Node(
                            name="yoloFilter_node",
                            package="autonomousDriving_ROSario",
                            executable="yolo_controller_bridge",
                            output = 'screen'
                            )
    
    delayed_node = TimerAction(
                                period=10.0,
                                actions=[yolo_bridge]
                               )
    

    l_d = LaunchDescription([yolo_recognition, delayed_node])
    return l_d