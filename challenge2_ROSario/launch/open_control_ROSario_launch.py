import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('challenge2_ROSario'),
                                    'config',
                                    'params.yaml'
    )

    puzzle_controller = Node(
                            name="controller_puzzlebot_ROSario",
                            package="challenge2_ROSario",
                            executable="open_control_node",
                            output = 'screen'
                            )
    
    puzzle_path = Node(
                        name="path_ROSario",
                        package="challenge2_ROSario",
                        executable="path_ROSario",
                        output = 'screen',
                        parameters=[config]
                      )

    l_d = LaunchDescription([puzzle_controller, puzzle_path])
    return l_d