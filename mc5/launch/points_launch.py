import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    # Robot 1: group1
    robot1_point_generator_node = Node(
        name="point_generator1",
        package='mc5',
        executable='point_generator',
        #namespace='group1',
        parameters=[{
            'figure':'square'
        }]
    )

    # Robot 2: group2
    robot2_point_generator_node = Node(
        name="point_generator2",
        package='mc5',
        executable='point_generator',
        namespace='group2',
        parameters=[{
            'figure':'square'
        }]
    )


    l_d = LaunchDescription([robot1_point_generator_node,])

    return l_d