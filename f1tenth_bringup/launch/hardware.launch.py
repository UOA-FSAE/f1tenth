import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'sensors.yaml'
    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )


    return LaunchDescription([

    ])
