import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('mpc-rbt-simulator')
    robot_description_path = os.path.join(package_dir, 'resources', 'tiago_plugin.urdf')
    tiago_description_path = os.path.join(package_dir, 'resources', 'tiago_ros.urdf')

    tiago_description_content = Command(['xacro ', tiago_description_path])
    tiago_description = {'robot_description': tiago_description_content}

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'mpc-rbt-warehouse.wbt')
    )

    tiago_driver = WebotsController(
        robot_name='tiago_base',
        namespace='tiago_base',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[tiago_description]
    )

    return LaunchDescription([
        webots,
        tiago_driver,
        robot_state_publisher_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])