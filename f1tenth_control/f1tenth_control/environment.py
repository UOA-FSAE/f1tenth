import rclpy
from rclpy.node import Node

from .launch_descriptions import launch_description_start_simulation, start_launch_description_process


class Environment(Node):
    def __init__(self, world_file: str, world_name: str):
        """

        :param world_file: Name of world or sdf file to load
        :param world_name: Name of world in sdf or world file
        """
        super().__init__(world_name)

        self.world_file = world_file
        self.world_name = world_name

        self.env_process = None

    def build(self):
        """
        Starts the simulation.
        """
        self.env_process = start_launch_description_process(
            launch_description_start_simulation,
            {'world_file': self.world_file}
        )


def main(args=None):
    rclpy.init(args=args)

    environment = Environment(
        world_file='empty.sdf',
        world_name='empty',
    )

    environment.build()
    rclpy.spin(environment)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    environment.destroy_node()
    rclpy.shutdown()
