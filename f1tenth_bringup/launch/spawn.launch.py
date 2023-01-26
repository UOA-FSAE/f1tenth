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
    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('name').perform(context)
    topic = LaunchConfiguration('topic').perform(context)

    cmd_exec = 'ros2 run ros_gz_sim create'

    # TODO: create topic bridges for the created model

    return [

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                f'/model/{name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/world/{world}/model/{name}/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
                f'/model/{name}/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
            ],
            remappings=[
                (f'/world/{world}/model/{name}/joint_state', '/joint_states'),
                (f'/model/{name}/tf', '/tf'),
            ]
        ),

        ExecuteProcess(
            cmd=[cmd_exec,
                 '-world', f"'{world}'",
                 '-name', f"'{name}'",
                 '-topic', f"'{topic}'",
                 ],
            output='screen',
            shell=True,
        )]


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

    pkg_f1tenth_bringup = get_package_share_directory('f1tenth_bringup')

    # TODO: move this into spawn_func and put / after name in frame_prefix
    rsp = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_bringup, 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        name_arg,
        topic_arg,

        rsp,

        OpaqueFunction(function=spawn_func)
    ])
