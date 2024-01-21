from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            output="screen",
            package="ros2_deskew_laser_scan",
            executable="ros2_deskew_laser_scan_node",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("ros2_deskew_laser_scan"),
                    "config", "ros2_deskew_laser_scan.yaml",
                ])],
        )
    ])
