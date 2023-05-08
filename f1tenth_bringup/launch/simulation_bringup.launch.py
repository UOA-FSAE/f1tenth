import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable

from launch_ros.actions import Node

import xacro


def spawn_func(context, *args, **kwargs):
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    description_pkg_path = os.path.join(get_package_share_directory('f1tenth_description'))
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file)

    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('name').perform(context)
    topic = LaunchConfiguration('topic').perform(context)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description.toxml(),
                'use_sim_time': False,
                'frame_prefix': name
            }]
        ),

        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-world', world,
                '-name', name,
                '-topic', topic,
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # f'/model/{name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/world/{world}/model/{name}/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
                f'/model/{name}/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',

            ],
            remappings=[
                (f'/world/{world}/model/{name}/joint_state', '/joint_states'),
                # (f'/model/{name}/tf', '/tf'),
            ]
        )
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        name='world',
        description='name of world'
    )

    name_arg = DeclareLaunchArgument(
        name='name',
        description='name of robot spawned'
    )

    topic_arg = DeclareLaunchArgument(
        name='topic',
        default_value='/robot_description',
        description='specifies xml topic to spawn from'
    )

    return LaunchDescription([
        world_arg,
        name_arg,
        topic_arg,

        OpaqueFunction(function=spawn_func)
    ])
