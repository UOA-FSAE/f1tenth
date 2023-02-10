from time import sleep

import rclpy
from rclpy.node import Node

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .launch_descriptions import launch_description_spawn_entity, start_launch_description_process

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Entity(Node):
    def __init__(self, name: str, world: str, urdf_robot_topic: str = '/robot_description'):
        super().__init__(name)

        self.name = name
        self.world = world
        self.urdf_robot_topic = urdf_robot_topic

        self.spawn_process = None
        self.odom = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pub_topic_cmd_vel = self.create_publisher(
            Twist,
            f'/model/{self.name}/cmd_vel',
            10
        )

        self.sub_mf_odom = Subscriber(
            self,
            Odometry,
            f'/model/{self.name}/odometry',
        )

        self.message_filter = ApproximateTimeSynchronizer(
            [self.sub_mf_odom],
            10,
            0.1
        )
        self.message_filter.registerCallback(self.message_filter_callback)

    def message_filter_callback(self, odom: Odometry):
        self.odom = odom

    def spawn(self):
        """
        This function needs to be called before interacting with the entity, This function spawns the entity in the
        simulation and allows for communication between the two
        """
        self.spawn_process = start_launch_description_process(
            launch_description_spawn_entity,
            {'name': self.name,
             'world': self.world,
             'topic': self.urdf_robot_topic}
        )

    def set_velocity(self, linear: float, angular: float):
        """
        Sets target velocity for the entity.

        :param linear: m/s
        :param angular: rad
        """
        velocity_msg = Twist()
        velocity_msg.angular.z = angular
        velocity_msg.linear.x = linear

        self.pub_topic_cmd_vel.publish(velocity_msg)

    def get_env(self):
        """
        Returns all observations of the environment that it currently has.
        :return: nav_msgs.msg.Odometry
        """
        return self.odom


def main(args=None):
    rclpy.init(args=args)

    entity = Entity(
        name='f1tenth_test_bot',
        world='empty'
    )

    entity.spawn()
    entity.get_logger().info("Entity spawned")
    sleep(2)

    angle = 0.5
    linear = 0.5
    entity.set_velocity(angle, linear)
    entity.get_logger().info(f"Entity given cmd_vel of {angle=}, {linear=}")

    while True:
        entity.get_logger().info(str(entity.get_env()))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    entity.destroy_node()
    rclpy.shutdown()
