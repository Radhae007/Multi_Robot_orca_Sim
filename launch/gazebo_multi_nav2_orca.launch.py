#!/usr/bin/env python3
"""
Multi-Robot Nav2 Simulation Launch Script with Decentralized P2P ORCA Proximity Switching.
Instantiates 4 TurtleBot3 robots sequentially in separate namespaces with Nav2, ORCA P2P nodes, and 4 dedicated RViz windows (1 per robot).
"""

import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_tb3_multi = get_package_share_directory('turtlebot3_multi_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav_launch_dir = os.path.join(pkg_tb3_multi, 'launch', 'nav2_bringup')

    # Simulation & Configuration Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    enable_orca = LaunchConfiguration('enable_orca', default='true')
    p2p_executable = LaunchConfiguration('p2p_executable', default='orca_p2p_node.py')

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_tb3_multi, 'params', 'nav2_orca_params.yaml')
    )
    # Can use custom Behaviour Trees by using including the path here.  
    bt_xml_file = LaunchConfiguration(
        'bt_xml_file',
        default=os.path.join(pkg_tb3_multi, 'behavior_trees', 'nav2_orca_switch_bt.xml')
    )

    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_tb3_multi, 'map', 'simple_warehouse.yaml')
    )

    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(pkg_tb3_multi, 'rviz', 'multi_nav2_default_view.rviz')
    )

    world_file = os.path.join(pkg_tb3_multi, 'worlds', 'simple_warehouse.world')

    # 4 Robots configuration (Namespaces, Initial Spawns, Unique Groot Ports)
    robots = [
        {
            'name': 'tb1', 'x_pose': '-2', 'y_pose': '-1.5', 'z_pose': '0.01', 'yaw': '0.0',
            'groot_publisher_port': '1666', 'groot_server_port': '1667'
        },
        {
            'name': 'tb2', 'x_pose': '-2', 'y_pose': '1.5', 'z_pose': '0.01', 'yaw': '0.0',
            'groot_publisher_port': '1668', 'groot_server_port': '1669'
        },
        {
            'name': 'tb3', 'x_pose': '2', 'y_pose': '-1.5', 'z_pose': '0.01', 'yaw': '3.14159',
            'groot_publisher_port': '1670', 'groot_server_port': '1671'
        },
        {
            'name': 'tb4', 'x_pose': '2', 'y_pose': '1.5', 'z_pose': '0.01', 'yaw': '3.14159',
            'groot_publisher_port': '1672', 'groot_server_port': '1673'
        },
    ]

    ld = LaunchDescription()

    # Declare Launch Arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_orca', default_value='true'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_tb3_multi, 'params', 'nav2_orca_params.yaml')))
    ld.add_action(DeclareLaunchArgument('bt_xml_file', default_value=os.path.join(pkg_tb3_multi, 'behavior_trees', 'nav2_orca_switch_bt.xml')))
    ld.add_action(DeclareLaunchArgument('map', default_value=os.path.join(pkg_tb3_multi, 'map', 'simple_warehouse.yaml')))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_tb3_multi, 'rviz', 'multi_nav2_default_view.rviz')))

    # 1. Gazebo Server & Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Common TF Remappings to prevent global namespace pollution
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 2. Shared Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}],
        remappings=tf_remappings
    )
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)

    # 3. Instantiate Multi-Robot Namespaces Sequentially
    urdf_file = os.path.join(pkg_tb3_multi, 'urdf', 'turtlebot3_waffle.urdf')
    last_action = None

    for robot in robots:
        ns = ['/' + robot['name']]

        # Robot State Publisher inside namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 30.0
            }],
            remappings=tf_remappings,
            arguments=[urdf_file]
        )

        # Spawn TurtleBot3 Model in Gazebo
        spawn_robot_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', ns,
                '-file', os.path.join(pkg_tb3_multi, 'models', 'turtlebot3_waffle', 'model.sdf'),
                '-robot_namespace', ns,
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', robot['z_pose'],
                '-Y', robot['yaw']
            ],
            output='screen'
        )

        # Decentralized P2P & Proximity Detector Node
        orca_p2p_node = Node(
            package='turtlebot3_multi_robot',
            executable=p2p_executable,
            name='orca_p2p_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'enable_orca': enable_orca,
                'd_threshold': 1.5,
                'broadcast_rate': 10.0,
                'peer_timeout': 2.0
            }]
        )

        # Nav2 Navigation Stack Bringup per Robot Namespace
        # Wrap in a GroupAction to remap its output velocity to cmd_vel_nav
        nav2_bringup_cmd = GroupAction([
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam': 'False',
                    'namespace': ns,
                    'use_namespace': 'true',
                    'map': '',
                    'map_server': 'False',
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                }.items()
            )
        ])

        # Sequential chaining to prevent node initialization drops
        if last_action is None:
            ld.add_action(rsp_node)
            ld.add_action(spawn_robot_cmd)
            ld.add_action(orca_p2p_node)
            ld.add_action(nav2_bringup_cmd)
        else:
            spawn_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[rsp_node, spawn_robot_cmd, orca_p2p_node, nav2_bringup_cmd],
                )
            )
            ld.add_action(spawn_event)

        last_action = spawn_robot_cmd

    # Start RViz and Initial Pose after all robots are spawned and Nav2 is up
    for robot in robots:
        ns = robot['name']
        
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'true',
                'use_sim_time': use_sim_time,
                'rviz_config': rviz_config_file,
            }.items()
        )

        yaw = float(robot['yaw'])
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose_msg = (
            f"{{header: {{frame_id: map}}, pose: {{pose: {{position: "
            f"{{x: {robot['x_pose']}, y: {robot['y_pose']}, z: 0.01}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw} }} }} }} }}"
        )
        init_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                f'/{ns}/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', pose_msg
            ],
            output='screen'
        )

        delayed_init_pose = TimerAction(
            period=15.0,
            actions=[init_pose_cmd]
        )

        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[delayed_init_pose, rviz_cmd],
            )
        )

        last_action = init_pose_cmd
        ld.add_action(post_spawn_event)

    return ld