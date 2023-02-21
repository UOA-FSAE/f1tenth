import subprocess

import rclpy
from rclpy.node import Node

from .launch_descriptions import launch_description_start_simulation, start_launch_description_process
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldReset, WorldControl

from rclpy.task import Future
from .entity import Entity
from uuid import uuid1


class Environment(Node):
    def __init__(self):
        super().__init__('Environment_Node')

        self.world_file = None
        self.world_name = None
        self.reset_pos = []

        self.list_of_entities = []

        self.env_process = None

    def config_world(self, world_file: str, world_name: str, reset_pos: list[(float, float)] = None):
        self.world_file = world_file
        self.world_name = world_name
        self.reset_pos = reset_pos

        return self

    def build(self):
        if not self.world_file and not self.world_name:
            raise Exception('Config world before build')

        self.env_process = start_launch_description_process(
            launch_description_start_simulation,
            {'world_file': self.world_file}
        )

    def create_entity(self, spawn_pos: (float, float) = None):
        if not spawn_pos and len(self.list_of_entities) == len(self.reset_pos):
            raise Exception('Did not give spawn position or there are more spawn positions than entities')

        entity = Entity(
            name='f1tenth',
            world=self.world_name,
            spawn_pos=(spawn_pos if spawn_pos else self.reset_pos[len(self.list_of_entities)])
        )

        self.list_of_entities.append(entity)
        entity.spawn()

        return entity

    def reset(self):
        # Moving Entity back to spawn pos
        for entity in self.list_of_entities:
            entity.move_entity()

    # Deprecated
    # def send_reset_request(self):
    #     self.reset_request.world_control = WorldControl()
    #
    #     self.reset_request.world_control.reset = WorldReset()
    #
    #     self.reset_request.world_control.reset.all = True
    #
    #     while not self.reset_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #
    #     self.reset_future = self.reset_client.call_async(self.reset_request)
    #     rclpy.spin_until_future_complete(self, self.reset_future)
    #
    #     return self.reset_future.result()


def main(args=None):
    rclpy.init(args=args)

    environment = Environment()
    environment.config_world(
        world_file='empty.sdf',
        world_name='empty'
    )
    environment.build()
    rclpy.spin(environment)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    environment.destroy_node()
    rclpy.shutdown()
