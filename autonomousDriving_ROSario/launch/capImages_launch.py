# Lanzador de modo camara | Final-Term Challenge
# Equipo ROSario

# Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    micro_ros_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'puzzlebot_ros', 'micro_ros_agent.launch.py'], #ros2 launch puzzlebot_ros micro_ros_agent.launch.py
                                    output='screen'
                                    )
    
    camera_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'ros_deep_learning', 'video_source.ros2.launch'], #ros2 launch ros_deep_learning video_source.ros2.launch
                                    output='screen'
                                    )

  
    capImages_node = Node(
                            name="capImages_node",
                            package="autonomousDriving_ROSario",
                            executable="yolo_capImages",
                            output = 'screen'
                            )
    
    
    delayed_nodes = TimerAction(
                                period=5.0,
                                actions=[capImages_node]
                               )

    l_d = LaunchDescription([camera_agent, delayed_nodes])
    return l_d