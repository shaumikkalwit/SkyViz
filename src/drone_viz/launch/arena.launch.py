from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('drone_viz')

    config_path = os.path.join(pkg_path, 'config', 'drones.yaml')
    urdf_xacro_file = os.path.join(pkg_path, 'urdf', 'drone_arena.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'arena.rviz')

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to spawn'
    )

    robot_description_cmd = Command([
        'xacro ', urdf_xacro_file, ' num_drones:=', LaunchConfiguration('num_drones')
    ])

    return LaunchDescription([
        num_drones_arg,

        Node(
            package='drone_viz',
            executable='drone_tf_broadcaster.py',
            name='drone_tf_broadcaster',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_cmd}]
        ),

        Node(
            package='drone_viz',
            executable='clicked_point_marker',
            name='clicked_point_marker',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
