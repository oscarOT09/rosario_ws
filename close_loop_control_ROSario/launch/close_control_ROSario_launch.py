import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('close_loop_control_ROSario'),
                                    'config',
                                    'params.yaml'
    )
    
    puzzle_localisation = Node(
                            name="localisation_node",
                            package="close_loop_control_ROSario",
                            executable="localisation_ROSario",
                            output = 'screen'
                            )

    puzzle_controller = Node(
                            name="controller_node",
                            package="close_loop_control_ROSario",
                            executable="controller_ROSario",
                            output = 'screen'
                            )
    
    puzzle_path = Node(
                        name="path_node",
                        package="close_loop_control_ROSario",
                        executable="pathGenerator_ROSario",
                        output = 'screen',
                        parameters=[config]
                      )

    l_d = LaunchDescription([puzzle_localisation,puzzle_controller, puzzle_path])
    return l_d