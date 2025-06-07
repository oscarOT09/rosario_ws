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
                                    cmd=['ros2', 'launch', 'puzzlebot_ros', 'camera_jetson.launch.py'], # ros2 launch puzzlebot_ros camera_jetson.launch.py #ros2 launch ros_deep_learning video_source.ros2.launch
                                    output='screen'
                                    )

  
    puzzle_controller = Node(
                            name="controller_node",
                            package="autonomousDriving_ROSario",
                            executable="controller_ROSario",
                            output = 'screen'
                            )
    
    line_detector = Node(
                            name="lineDetector_node",
                            package="autonomousDriving_ROSario",
                            executable="lineDetector_ROSario",
                            output = 'screen'
                            )
    
    trafficLight_detector = Node(
                            name="trafficLightDetector_node",
                            package="autonomousDriving_ROSario",
                            executable="trafficLightDetector_ROSario",
                            output = 'screen'
                            )
    
    frames_pub = Node(
                            name="framesPublisher_node",
                            package="autonomousDriving_ROSario",
                            executable="frames_pc_publisher",
                            output = 'screen'
                            )
    
    '''camera_recorder = Node(
                            name="videoRecorder_node",
                            package="autonomousDriving_ROSario",
                            executable="camera_recorder",
                            output = 'screen'
                            )'''
    
    delayed_camera = TimerAction(
                                period=5.0,
                                actions=[camera_agent]
                               )
    
    delayed_puzzlebot = TimerAction(
                                period=7.0,
                                actions=[line_detector, puzzle_controller, frames_pub]
                               )

    l_d = LaunchDescription([micro_ros_agent, delayed_camera, delayed_puzzlebot])
    return l_d