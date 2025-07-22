from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('drone_viz'),
        'urdf',
        'drone_arena.urdf'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('drone_viz'),
        'rviz',
        'arena.rviz'
    )

    print(f"Using URDF file: {urdf_file}")
    print(f"Using RViz config: {rviz_config_file}")

    # Read URDF content safely
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
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
