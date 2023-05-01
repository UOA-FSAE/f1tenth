# create simple ros2 subscriber and publisher

import rclpy
from rclpy.node import Node
from f1tenth_msgs.msg import Available, Configure

from statemachine import StateMachine, State


class StateController(Node, StateMachine):
    """
    StateController class

    States:
        - unconfigured
        - active

    Transitions:
        - configure
        - deactivate

    During unconfgured state:
        If it is sent a Available.msg what has get_available_cars = True on the "/available_cars"
        then it will publish its ID on the topic in reply

        When sent Configure.msg on "/configure_car" topic it will change to active state and config as
        the message says and transition to active state
    """

    # States
    unconfigured = State('unconfigured', initial=True)
    active = State('active')

    # Transitions
    configure = unconfigured.to(active)
    deactivate = active.to(unconfigured)

    def __init__(self):
        super().__init__('state_controller')

    # Transitions
    @unconfigured.enter
    def on_enter_unconfigured(self):
        # Create topics and callbacks
        self.get_logger().info("StateController: Entering unconfigured state")

        # Subscribers
        self.sub_available_cars = self.create_subscription(
            Available,
            '/available_cars',
            self.available_cars_callback,
            10
        )
        self.sub_configure_car = self.create_subscription(
            Configure,
            '/configure_car',
            self.configure_car_callback,
            10
        )

        # Publishers
        self.pub_car_id = self.create_publisher(
            Configure,
            '/car_id',
            10
        )


    @unconfigured.exit
    def on_exit_unconfigured(self):
        # Destroys subscribers and publishers
        self.destroy_subscription(self.sub_available_cars)
        self.destroy_subscription(self.sub_configure_car)
        self.destroy_publisher(self.pub_car_id)

    @active.enter
    def on_enter_active(self):
        pass

    @active.exit
    def on_exit_active(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    state_controller = StateController()
    rclpy.spin(state_controller)
    state_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
