import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # ---------- RSP ----------

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    description_pkg_path = os.path.join(get_package_share_directory('f1tenth_description'))
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file)

    rsp_params = {'robot_description': robot_description.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )

    # ---------- JSP ----------
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp_gui_arg = DeclareLaunchArgument(
        name='use_jsp_gui',
        default_value='false',
        description='Loads joint state publisher gui if true'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_jsp_gui),
    )

    # ---------- RVIZ ---------
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Loads Rviz2 if True')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[f'-d {os.path.join(description_pkg_path, "rviz", "config.rviz" )}'],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Launch configs
        use_sim_time_arg,
        use_jsp_gui_arg,
        use_rviz_arg,

        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
