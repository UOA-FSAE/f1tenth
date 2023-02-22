from time import sleep

import rclpy
from rclpy.node import Node

from message_filters import Subscriber, ApproximateTimeSynchronizer

from .launch_descriptions import launch_description_spawn_entity, start_launch_description_process

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from rclpy.executors import SingleThreadedExecutor
from threading import Thread

import random
import time
import subprocess


class Entity(Node):
    def __init__(self, name: str, world: str, spawn_pos, urdf_robot_topic: str = '/robot_description'):
        super().__init__(name)

        self.name = str(name)
        self.world = world
        self.spawn_pos = spawn_pos
        self.urdf_robot_topic = urdf_robot_topic

        self.spawn_process = None
        self.odom = None

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
            0.1,
        )
        self.message_filter.registerCallback(self.message_filter_callback)

    def pose_setter(self, odom: Pose):
        self.odom = odom

    def spin_up(self):
        self.executor_thread = SingleThreadedExecutor()

        def run_func():
            self.executor_thread.add_node(self)
            self.executor_thread.spin()
            self.executor_thread.remove_node(self)

        self.dedicated_listener_thread = Thread(target=run_func)
        self.dedicated_listener_thread.start()

    def message_filter_callback(self, odom: Odometry):
        self.odom = odom

    def spawn(self, spin_thread=True):
        self.spawn_process = start_launch_description_process(
            launch_description_spawn_entity,
            {'name': self.name,
             'world': self.world,
             'topic': self.urdf_robot_topic}
        )

        if spin_thread:
            self.spin_up()

    def move_entity(self, x=None, y=None, z=0):

        if not x:
            x = self.spawn_pos[0]

        if not y:
            y = self.spawn_pos[1]

        x = float(x)
        y = float(y)
        z = float(z)

        subprocess.Popen(
            # gz service - s /world/empty/set_pose - -reqtype gz.msgs.Pose - -reptype gz.msgs.Boolean - -timeout 1000
            # - -req 'name: "blue_car", position: {x: 0, y: 0, z: 0.5}'
            [f"gz service -s /world/{self.world}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"{self.name}\", position: {{x: {str(x)}, y: {str(y)}, z:{str(z)}}}'"],
            shell=True
        )

    def set_velocity(self, linear: float, angular: float):
        velocity_msg = Twist()
        velocity_msg.angular.z = angular
        velocity_msg.linear.x = linear

        self.pub_topic_cmd_vel.publish(velocity_msg)

    def get_data(self):
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
