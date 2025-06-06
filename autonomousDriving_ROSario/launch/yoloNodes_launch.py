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
    
    yolo_filter = Node(
                            name="yoloFilter_node",
                            package="autonomousDriving_ROSario",
                            executable="yolo_controller_bridge",
                            output = 'screen'
                            )

    l_d = LaunchDescription([yolo_recognition, yolo_filter])
    return l_d