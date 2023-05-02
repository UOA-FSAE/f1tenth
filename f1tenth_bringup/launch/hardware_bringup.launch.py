import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration


def generate_launch_description():
    '''
    inputs:
        name: name of f1tenth car
        vesc_config: overides defult config (this contains the topic names as well)
    '''
    # launch config
    name_arg = DeclareLaunchArgument(
        name='name',
        description='set namespace of car'
    )

    # launch vesc
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
    )
    vesc_config_la = DeclareLaunchArgument(
        name="vesc_config",
        default_value=vesc_config,
        description="VESC yaml configuration file.",
    )
    vesc_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration("vesc_config")],
        namespace=LaunchConfiguration('name')
    )

    # Ackermann to vesc
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        namespace=LaunchConfiguration('name'),
        parameters=[{"speed_to_erpm_gain": "4614.0",
                     "speed_to_erpm_offset": "0.0",
                     "steering_angle_to_servo_gain": "1.0",
                     "steering_angle_to_servo_offset": "0.5304"
                     }.items()
                    ]
    )

    # Vesc to odom
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace=LaunchConfiguration('name'),
        parameters=[{"odom_frame": "odom",
                     "base_frame": "base_link",
                     "speed_to_erpm_gain": "1.0",
                     "speed_to_erpm_offset": "0.0",
                     "use_servo_cmd_to_calc_angular_velocity": "true",
                     "steering_angle_to_servo_gain": "1.0",
                     "steering_angle_to_servo_offset": "0.0",
                     "wheelbase": "0.2",
                     "publish_tf": "true",
                     }.items()
                    ]
    )

    return LaunchDescription([
        # launch config
        name_arg,
        vesc_config_la,

        # launch nodes
        vesc_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node
    ])
