import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    '''config = os.path.join(
        get_package_share_directory('line_follower_ROSario'),
                                    'config',
                                    'params.yaml'
    )'''
    
    micro_ros_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'puzzlebot_ros', 'micro_ros_agent.launch.py'], #ros2 launch puzzlebot_ros micro_ros_agent.launch.py
                                    output='screen'
                                    )
    
    camera_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'ros_deep_learning', 'video_source.ros2.launch'], #ros2 launch ros_deep_learning video_source.ros2.launch
                                    output='screen'
                                    )

  
    puzzle_controller = Node(
                            name="controller_node",
                            package="line_follower_ROSario",
                            executable="controller_ROSario",
                            output = 'screen'
                            )
    
    line_detector = Node(
                            name="lineDetector_node",
                            package="line_follower_ROSario",
                            executable="lineDetector_ROSario",
                            output = 'screen'
                            )
    
    puzzle_localisation = Node(
                            name="localisation_node",
                            package="line_follower_ROSario",
                            executable="localisation_ROSario",
                            output = 'screen'
                            )
    
    trafficLight_detector = Node(
                            name="trafficLightDetector_node",
                            package="line_follower_ROSario",
                            executable="trafficLightDetector_ROSario",
                            output = 'screen'
                            )
    
    camera_recorder = Node(
                            name="videoRecorder_node",
                            package="line_follower_ROSario",
                            executable="camera_recorder",
                            output = 'screen'
                            )
    
    delayed_camera = TimerAction(
                                period=5.0,
                                actions=[camera_agent]
                               )
    
    delayed_puzzlebot = TimerAction(
                                period=7.0,
                                actions=[line_detector, trafficLight_detector, puzzle_controller,camera_recorder]
                               )

    l_d = LaunchDescription([micro_ros_agent, delayed_camera, delayed_puzzlebot])
    return l_d