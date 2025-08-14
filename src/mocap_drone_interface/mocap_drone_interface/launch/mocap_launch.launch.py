from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    nodes = []

    package_share_dir = Path(get_package_share_directory('mocap_drone_interface'))
    config_file = package_share_dir / 'config' / 'params.yaml'

    # nodes.append(
    #     Node(
    #         package = 'mocap_drone_interface',
    #         executable = 'posevis',
    #         parameters=[str(config_file)],
    #         output = 'screen'
    #     )
    # )
    nodes.append(
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            output = 'screen'
        )
    )
    nodes.append(
        Node(
            package = 'mocap_drone_interface',
            executable = 'service',
            parameters=[str(config_file)],
            output = 'screen'
        )
    )

    return LaunchDescription(nodes)