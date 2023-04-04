import os

from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    '''
    inputs:
        world_sdf : loads world sdf
        simulation_speed: hz simulation is run at defult is 1000 / 1x speed
    '''

    return LaunchDescription([
    ])
