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

  
    openLoopNode = Node(
                            name="openLoop_node",
                            package="autonomousDriving_ROSario",
                            executable="open_loop_actions",
                            output = 'screen'
                            )
    
    delayed_node = TimerAction(
                                period=5.0,
                                actions=[openLoopNode]
                               )
    

    l_d = LaunchDescription([micro_ros_agent, delayed_node])
    return l_d