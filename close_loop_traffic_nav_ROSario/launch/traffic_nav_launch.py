import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('close_loop_traffic_nav_ROSario'),
                                    'config',
                                    'params.yaml'
    )
    
    micro_ros_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'puzzlebot_ros', 'micro_ros_agent.launch.py'], #ros2 launch puzzlebot_ros micro_ros_agent.launch.py
                                    output='screen'
                                    )
    
    camera_agent = ExecuteProcess(
                                    cmd=['ros2', 'launch', 'ros_deep_learning', 'video_source.ros2.launch'], #ros2 launch ros_deep_learning video_source.ros2.launch
                                    output='screen'
                                    )

    puzzle_localisation = Node(
                            name="localisation_node",
                            package="close_loop_traffic_nav_ROSario",
                            executable="localisation_ROSario",
                            output = 'screen'
                            )

    puzzle_controller = Node(
                            name="controller_node",
                            package="close_loop_traffic_nav_ROSario",
                            executable="controller_ROSario",
                            output = 'screen'
                            )
    
    puzzle_path = Node(
                        name="path_node",
                        package="close_loop_traffic_nav_ROSario",
                        executable="path_generator_ROSario",
                        output = 'screen',
                        parameters=[config]
                      )
    
    colorIdentificator = Node(
                                name="color_identificator",
                                package="close_loop_traffic_nav_ROSario",
                                executable="colorIdentificator",
                                output = 'screen'
                              )
    
    delayed_camera = TimerAction(
                                period=5.0,
                                actions=[camera_agent]
                               )
    
    delayed_puzzlebot = TimerAction(
                                period=10.0,
                                actions=[puzzle_path, puzzle_localisation, puzzle_controller, colorIdentificator]
                               )

    l_d = LaunchDescription([micro_ros_agent, delayed_camera, delayed_puzzlebot])
    return l_d