import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
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
    pkg_vesc_ackermann = get_package_share_directory('vesc_ackermann')
    # launch vesc
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
        ),
		Node(
		    package='vesc_driver',
		    executable='vesc_driver_node',
		    name='vesc_driver_node',
		    namespace='f1tenth',
		    parameters=[LaunchConfiguration("config")]
		),
		PushRosNamespace(
        	namespace='f1tenth',
		    launch_descrption=[
				IncludeLaunchDescription(
					launch_description_source=XMLLaunchDescriptionSource(
						os.path.join(pkg_vesc_ackermann, 'launch', 'ackermann_to_vesc_node.launch.xml'),
				)),
				IncludeLaunchDescription(
					launch_description_source=XMLLaunchDescriptionSource(
						os.path.join(pkg_vesc_ackermann, 'launch', 'vesc_to_odom_node.launch.xml'),	
				))
		    ],
        )
    ])
