import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node 
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_f1tenth_description = get_package_share_directory('f1tenth_description')

    # TODO: remove the hardcoding of topic name
    car_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            f'/model/f1tenth/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/f1tenth/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan', # TODO: find way to publish lidar to dynamic topic => of form /model/<name>/lidar
        ]
    )
        # f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        # f'/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        # f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        # # f'/model/{name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        # f'/world/{world}/model/{name}/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        # f'/model/{name}/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
    
    xacro_file = os.path.join(pkg_f1tenth_description, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # TODO: remove the hardcoding of car name
    f1tenth_spawn = Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-name', 'f1tenth',
                '-string', robot_description,
            ],
            output='screen'
        )

    return LaunchDescription([
        car_bridge,
        f1tenth_spawn
    ])
