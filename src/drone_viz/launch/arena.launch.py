from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_path = get_package_share_directory('drone_viz')

    config_path = os.path.join(pkg_path, 'config', 'drones.yaml')
    urdf_xacro_file = os.path.join(pkg_path, 'urdf', 'drone_arena.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'arena.rviz')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    for drone in config['drones']:
        print(f"Name: {drone['name']}")
        print(f"Pose Topic: {drone['pose_topic']}")
        print(f"Mesh: {drone['mesh']}")
        print("---")


    urdf_xacro_file = os.path.join(pkg_path, 'urdf', 'drone_arena.xacro')
    robot_description = Command(['xacro ', urdf_xacro_file])


    drones = config['drones']

    nodes = []

    # Launch tf broadcasters per drone
    for drone in drones:
        nodes.append(
            Node(
                package='drone_viz',
                executable='drone_tf_broadcaster.py',
                name=f"{drone['name']}_tf_broadcaster",
                output='screen',
                parameters=[{
                    'drone_name': drone['name'],
                    'pose_topic': drone['pose_topic']
                }]
            )
        )
        print(drone)

    # Add robot state publisher
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        )
    )

    # Add clicked point marker
    nodes.append(
        Node(
            package='drone_viz',
            executable='clicked_point_marker',
            name='clicked_point_marker',
            output='screen'
        )
    )

    # Add RViz
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    )

    return LaunchDescription(nodes)