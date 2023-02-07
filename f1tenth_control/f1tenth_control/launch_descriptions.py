import os
from multiprocessing import Process

from ament_index_python.packages import get_package_share_directory

from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def start_launch_description_process(launch_description, args):
    ls = LaunchService()

    ls.include_launch_description(
        launch_description(args)
    )

    process = Process(target=ls.run)
    process.daemon = True
    process.start()

    return process


def launch_description_spawn_entity(args) -> LaunchDescription:
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    spawn_launch_description = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'spawn.launch.py')
        ),
        launch_arguments={
            'world': args['world'],
            'name': args['name'],
            'topic': args['topic']
        }.items()
    )

    return LaunchDescription([
        spawn_launch_description
    ])


def launch_description_start_simulation(args) -> LaunchDescription:
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    sim_launch_description = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'sim.launch.py')
        )
    )

    return LaunchDescription([
        sim_launch_description
    ])
