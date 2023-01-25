from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction


# TODO: add documentation

def spawn_func(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    name = LaunchConfiguration('name').perform(context)
    topic = LaunchConfiguration('topic').perform(context)

    cmd_exec = 'ros2 run ros_gz_sim create'

    # TODO: create topic bridges for the created model

    return [ExecuteProcess(
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
        default_value='',
        description='leave blank unless loading into specified world'
    )

    name_arg = DeclareLaunchArgument(
        name='name',
        default_value='',
        description='name of f1tenth car spawned'
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
