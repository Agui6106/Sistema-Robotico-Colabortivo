import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('mc6'),
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
    


    robot1_controller_node = Node(name="controller1",
                            package='mc6',
                            executable='controller',
                            #namespace='group1',
                            parameters=[{
                                'kw': 1.0,
                                'kv': 1.0,
                                'use_sim_time': True,
                            }],

                        )
    
    
    robot1_puzzlebot_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('puzzlebot_gazebo'),
                'launch',
                'bringup_simulation_simple_launch.py'
            )
        )
    )

    robot1_localisation_node = Node(
        name="localisation1",
        package='mc6',
        executable='localisation',
        #namespace='group1',
        parameters=[{
            'r': 0.05,
            'L': 0.19,
            'use_sim_time': True,
            }]
    )

    robot1_wall_following_node = Node(
        name="wall_following1",
        package='mc6',
        executable='wall_following_bug2',
        #namespace='group1',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    robot1_joint_pub_node = Node(
        name="puzzlebotsim1",
        package='mc6',
        executable='joint_pub',
        #namespace='group1',
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
                            #parameters=[{'frame_prefix': 'group1/','robot_description': robot_desc}],
                            parameters=[{'robot_description': robot_desc}],
                            #namespace='group1',
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
    

    l_d = LaunchDescription([robot1_puzzlebot_sim_launch, 
                            robot1_controller_node,
                            robot1_localisation_node,
                            robot1_wall_following_node,
                            ])

    return l_d