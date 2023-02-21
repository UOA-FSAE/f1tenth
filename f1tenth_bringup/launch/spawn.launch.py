import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription

from launch_ros.actions import Node


# TODO: add documentation
# TODO: clean up code

def spawn_func(context, *args, **kwargs):
    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('name').perform(context)
    topic = LaunchConfiguration('topic').perform(context)

    rsp = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'frame_prefix': f'{name}/',
        }.items(),
    )

    return [
        rsp,

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
