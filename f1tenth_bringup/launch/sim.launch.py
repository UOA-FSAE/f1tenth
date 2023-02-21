import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


# TODO: add documentation

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # TODO: add f1tenth_description to resource path for gazebo
    # TODO: add world launch argument
    gz_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r empty.sdf',
        }.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
        ]
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge
    ])
