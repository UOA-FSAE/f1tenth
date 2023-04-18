from launch import LaunchDescription 
from launch_ros.actions import Node 
import launch

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='simulation',
            executable='gz_services',
            output='screen',
            arguments = ['world_path', 'world_name']
        )
])