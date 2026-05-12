import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('mc4'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '1', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )
    
    # Robot 1: group1
    robot1_point_generator_node = Node(
        name="point_generator1",
        package='mc4',
        executable='point_generator',
        namespace='group1',
        parameters=[{
            'figure':'square'
        }]
    )

    robot1_controller_node = Node(name="controller1",
                            package='mc4',
                            executable='controller',
                            namespace='group1',
                            parameters=[{
                                'kw': 1.0,
                                'kv': 1.0
                            }]
                        )
    
    robot1_puzzlebot_sim_node = Node(name="puzzlebot_sim1",
                            package='mc4',
                            executable='puzzlebot_sim',
                            namespace='group1',
                            parameters=[{
                                'r': 0.05,
                                'L': 0.19
                            }]
                        )

    robot1_localisation_node = Node(
        name="localisation1",
        package='mc4',
        executable='localisation',
        namespace='group1',
        parameters=[{
            'r': 0.05,
            'L': 0.19        
            }]
    )

    robot1_joint_pub_node = Node(
        name="puzzlebotsim1",
        package='mc4',
        executable='joint_pub',
        namespace='group1',
        parameters=[{
            'init_pose_x': 0.0,
            'init_pose_y': 0.0,
            'init_pose_z': 0.0,
            'init_pose_yaw': 0.0,
            'init_pose_pitch': 0.0,
            'init_pose_roll': 0.0,
            'odom_frame':'odom'
        }]
    )

    robot1_state_pub_node = Node(
                            name='robot_state_publisher',
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            output='screen',
                            parameters=[{'frame_prefix': 'group1/','robot_description': robot_desc}],
                            namespace='group1',
                            )
    

    # Robot 2: group2
    robot2_point_generator_node = Node(
        name="point_generator2",
        package='mc4',
        executable='point_generator',
        namespace='group2',
        parameters=[{
            'figure':'star'
        }]
    )

    robot2_controller_node = Node(name="controller2",
                            package='mc4',
                            executable='controller',
                            namespace='group2',
                            parameters=[{
                                'kw': 1.5,
                                'kv': 1.5
                            }]
                        )
    
    robot2_puzzlebot_sim_node = Node(name="puzzlebot_sim2",
                            package='mc4',
                            executable='puzzlebot_sim',
                            namespace='group2',
                            parameters=[{
                                'r': 0.05,
                                'L': 0.19
                            }]
                        )
    
    robot2_localisation_node = Node(
        name="localisation2",
        package='mc4',
        executable='localisation',
        namespace='group2',
        parameters=[{
            'r': 0.05,
            'L': 0.19        
            }]
    )

    robot2_joint_pub_node = Node(
        name="puzzlebotsim2",
        package='mc4',
        executable='joint_pub',
        namespace='group2',
        parameters=[{
            'init_pose_x': 1.0,
            'init_pose_y': 2.0,
            'init_pose_z': 0.0,
            'init_pose_yaw': 0.0,
            'init_pose_pitch': 0.0,
            'init_pose_roll': 0.0,
            'odom_frame':'odom'
        }]
    )

    robot2_state_pub_node = Node(
                            name='robot_state_publisher',
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            output='screen',
                            parameters=[{'frame_prefix': 'group2/','robot_description': robot_desc}],
                            namespace='group2',
                            )
    

    
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
    
    rqt_graph_node = Node(name='rqt_graph',
                    package='rqt_graph',
                    executable='rqt_graph'
                    )
    

    l_d = LaunchDescription([static_transform_node, 

                            robot1_state_pub_node,
                            #robot1_point_generator_node,
                            robot1_controller_node,
                            robot1_puzzlebot_sim_node,
                            robot1_localisation_node,
                            robot1_joint_pub_node,

                            robot2_state_pub_node,
                            #robot2_point_generator_node,
                            robot2_controller_node,
                            robot2_puzzlebot_sim_node,
                            robot2_localisation_node,
                            robot2_joint_pub_node,

                            rqt_tf_tree_node, 
                            rqt_plot_node, 
                            rqt_graph_node,
                            rviz_node])

    return l_d