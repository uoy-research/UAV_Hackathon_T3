import atexit
import sys
from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class Teleop(Node, ABC):
    def __init__(self):
        atexit.register(self._emergency_stop)
        Node.__init__(self, "keyboard_teleop")

        self.declare_parameter("twist_stamped_enabled", False)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("linear1_max", 1.0)
        self.declare_parameter("linear2_max", 1.0)
        self.declare_parameter("linear3_max", 1.0)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("publish_rate", 10.0)
        self.LINEAR1_MAX = self.get_parameter("linear1_max").value
        self.LINEAR2_MAX = self.get_parameter("linear2_max").value
        self.LINEAR3_MAX = self.get_parameter("linear3_max").value

        self.ANGULAR_MAX = self.get_parameter("angular_max").value

        self._robot_base_frame = self.get_parameter("robot_base_frame").value

        if self.get_parameter("twist_stamped_enabled").value:
            self.publisher_ = self.create_publisher(
                TwistStamped, "cmd_vel", qos_profile_system_default
            )
            self._make_twist = self._make_twist_stamped
        else:
            self.publisher_ = self.create_publisher(
                Twist, "cmd_vel", qos_profile_system_default
            )
            self._make_twist = self._make_twist_unstamped
        rate = 1 / self.get_parameter("publish_rate").value
        self.create_timer(rate, self._publish)
        self.linear1 = 0.0
        self.linear2 = 0.0
        self.linear3 = 0.0
        self.angular = 0.0

    @abstractmethod
    def update_twist(self, *args):
        pass

    def write_twist(self, linear1=None, linear2=None, linear3=None,angular=None):
        if linear1 is not None:
            if abs(linear1) <= self.LINEAR1_MAX:
                self.linear1 = linear1
            else:
                self.get_logger().error(
                    f"Trying to set a linearx speed {linear1} outside of allowed range of [{-self.LINEAR1_MAX}, {self.LINEAR1_MAX}]"
                )
        if linear2 is not None:
            if abs(linear2) <= self.LINEAR2_MAX:
                self.linear2 = linear2
            else:
                self.get_logger().error(
                    f"Trying to set a lineary speed {linear2} outside of allowed range of [{-self.LINEAR2_MAX}, {self.LINEAR2_MAX}]"
                )
        if linear3 is not None:
            if abs(linear3) <= self.LINEAR3_MAX:
                self.linear3 = linear3
            else:
                self.get_logger().error(
                    f"Trying to set a linearz speed {linear3} outside of allowed range of [{-self.LINEAR3_MAX}, {self.LINEAR3_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
                )
        self._update_screen()

    def _make_twist_unstamped(self, linear1, linear2, linear3, angular):
        twist = Twist()
        twist.linear.x = linear1
        twist.linear.y = linear2
        twist.linear.z = linear3
        twist.angular.z = angular
        return twist

    def _make_twist_stamped(self, linear1, linear2, linear3, angular):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self._robot_base_frame
        twist_stamped.twist = self._make_twist_unstamped(linear1, linear2, linear3, angular)
        return twist_stamped

    def _publish(self):
        twist = self._make_twist(self.linear1, self.linear2, self.linear3, self.angular)
        self.publisher_.publish(twist)

    def _update_screen(self):
        sys.stdout.write(f"LinearX: {self.linear1:.2f}, LinearY: {self.linear2:.2f}, LinearZ: {self.linear3:.2f}, Angular: {self.angular:.2f}\r")

    def _emergency_stop(self):
        self.publisher_.publish(self._make_twist(0.0, 0.0, 0.0, 0.0))
