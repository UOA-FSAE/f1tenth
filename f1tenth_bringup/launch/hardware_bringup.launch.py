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
        vesc_config : overide
        sensor_config: overide
        topic_config: lables all topics
    '''

    # launch vesc
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vesc_driver'),
                'launch',
                'vesc_driver_node.launch.py'
            ])
        ])
    )

    ackermann_to_vesc_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vesc_ackermann'),
                'launch',
                'ackermann_to_vesc_node.launch.xml'
            ])
        ])
    )

    vesc_to_odom_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vesc_ackermann'),
                'launch',
                'vesc_to_odom_node.launch.xml'
            ])
        ])
    )

    return LaunchDescription([
        ackermann_to_vesc_launch,
        vesc_to_odom_launch,
        vesc_launch,
    ])
