import os

import rclpy
from rclpy.node import Node

from ament_index_python import get_package_share_directory

from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(world_file: str) -> LaunchDescription:
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    sim_launch_description = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'launch', 'sim.launch.py')
        )
    )

    return LaunchDescription([
        sim_launch_description
    ])


class Environment(Node):
    def __init__(self, world_file: str, world_name: str):
        super().__init__(world_name)

        self.world_file = world_file
        self.world_name = world_name

    def build(self):
        ls = LaunchService()

        ls.include_launch_description(
            generate_launch_description(self.world_file)
        )

        ls.run_async()
