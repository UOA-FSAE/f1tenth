import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    gz_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'))
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            # TODO: Submit issue to https://github.com/gazebosim/ros_gz/issues cuz this shit aint working
            # 'config_file': os.path.join(pkg_f1tenth_bringup, 'config', 'ros_gz_example_bridge.yaml')

            # TODO: Bridge cmd and other topics
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
    ])
