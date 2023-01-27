import os

from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(name: str, world: str, topic: str) -> LaunchDescription:
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    spawn_launch_description = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'launch', 'spawn.launch.py')
        ),
        launch_arguments={
            'world': world,
            'name': name,
            'topic': topic
        }.items()
    )

    return LaunchDescription([
        spawn_launch_description
    ])


class Entity(Node):
    def __init__(self, name: str, world: str, topic: str):
        super().__init__(name)

        self.name = name
        self.world = world
        self.topic = topic

    def spawn(self):
        ls = LaunchService()

        ls.include_launch_description(
            generate_launch_description(self.name)
        )

        ls.run_async()
