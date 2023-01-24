import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    rsp = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'use_jsp_gui': 'true',
        }.items(),
    )

    gz_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': 'empty.sdf',
        }.items()
    )

    return LaunchDescription([
        rsp,
        gz_sim,
    ])
