import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_simulator')
    robot_description_path = os.path.join(package_dir, 'resources', 'tiago_plugin.urdf')
    tiago_description_path = os.path.join(package_dir, 'resources', 'tiago_model_ros.urdf')
    map_file_path = os.path.join(package_dir, 'maps', 'map.yaml')

    tiago_description_content = Command(['xacro ', tiago_description_path])
    tiago_description = {'robot_description': tiago_description_content}

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'mpc-rbt-warehouse.wbt'),
        ros2_supervisor=True
    )

    tiago_driver = WebotsController(
        robot_name='tiago_base',
        # namespace='tiago_base',
        parameters=[
            {'robot_description': robot_description_path},
            {'cmd_vel_timeout_seconds': 0.1},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[tiago_description]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        # namespace=namespace,
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': map_file_path},
            {'topic_name': "map"},
        ]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        # namespace=namespace,
        name='lifecycle_manager_map_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'autostart': True},
            {'use_sim_time': True},
            {'node_names': ['map_server']},
        ]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        tiago_driver,
        robot_state_publisher_node,
        map_server_node,
        lifecycle_manager_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
