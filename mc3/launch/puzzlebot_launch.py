import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------------------- #
    # ------- Dependencies ------- #
    # ---------------------------- #

    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('mc3'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )
    
    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '1', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )
    
    # --------------- #
    # -- Sim Nodes -- #
    # --------------- #      
    puzzlebot_sim_node = Node(name="puzzlebot_sim",
                              package='mc3',
                              executable='puzzlebot_sim',
                              )

    localisation_node = Node(name="localisation",
                              package='mc3',
                              executable='localisation',
                              )
    
    puzzlebot_node = Node(name="joint_pub",
                            package='mc3',
                            executable='joint_pub',
                            )
    
    controller_node = Node(name="controller",
                            package='mc3',
                            executable='controller',
                            )
    
    path_gen_node = Node(name="path_generator",
                         package="mc3",
                         executable="path_generator")

    # ---------------------------- #
    # -- Nodos de visualizacion -- #
    # ---------------------------- #
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2'
                    )
    
    rqt_plot_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot'
                    )


    l_d = LaunchDescription([robot_state_pub_node, static_transform_node,
                             puzzlebot_sim_node, localisation_node, puzzlebot_node, controller_node, path_gen_node,
                             rqt_plot_node, rqt_tf_tree_node, rviz_node
                            ])

    return l_d