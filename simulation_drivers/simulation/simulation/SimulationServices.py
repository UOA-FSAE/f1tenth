import sys
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
from ros_gz_interfaces.msg import Entity, EntityFactory


class SimulationServices(Node):
    def __init__(self, world_name):
        super().__init__('simulation_services')
        
        self.world_name = world_name

        self.spawn_client = self.create_client(
            SpawnEntity,
            f'world/{self.world_name}/create',
        )

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        self.spawn_future = Future()

        # Delete Service --------------------------------------
        self.delete_client = self.create_client(
            DeleteEntity,
            f'world/{self.world_name}/remove',
        )
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('delete service not available, waiting again...')

        self.delete_future = Future()

        # Set Pose Service --------------------------------------
        self.set_pose_client = self.create_client(
            SetEntityPose,
            f'world/{self.world_name}/set_pose',
        )
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('delete service not available, waiting again...')

        self.delete_future = Future()


if __name__ == '__main__':

    main()
