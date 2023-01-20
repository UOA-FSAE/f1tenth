import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp_gui = LaunchConfiguration('use_rjp_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('f1tenth_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=['-d' + os.path.join(get_package_share_directory(pkg_path), 'rviz', rviz_config)]
    )

    launch_list = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_rjp_gui',
            default_value='False',
            description='Loads joint state publisher gui if true'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Loads Rviz if True'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value='default.rviz',
            description='rviz2 config file'),

        robot_state_publisher,
    ]

    if use_jsp_gui:
        launch_list.append(joint_state_publisher_gui)
    if use_rviz:
        launch_list.append(rviz)

    # Launch!
    return LaunchDescription(launch_list)
