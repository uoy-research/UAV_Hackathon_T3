import os
import signal

import rclpy
from pynput.keyboard import Key, Listener

from keyboard_teleop.teleop import Teleop


class IncrementalKeyTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.key_listener = Listener(
            on_press=self.update_twist,
        )
        self.key_listener.start()
        self.declare_parameter("step", 0.2)
        self.STEP = self.get_parameter("step").value
        self.keys_bindings = {
            "w": (self.STEP, 0, 0, 0),
            "s": (-self.STEP, 0, 0, 0),
            "a": (0, 0, 0, self.STEP),
            "d": (0, 0, 0, -self.STEP),
            "q": (0, self.STEP, 0, 0),
            "z": (0, -self.STEP, 0, 0),
            "e": (0, 0, self.STEP, 0),
            "c": (0, 0, -self.STEP, 0),
        }
        self.special_keys_bindings = {
            Key.up: (self.STEP, 0, 0, 0),
            Key.down: (-self.STEP, 0, 0),
            Key.left: (0, 0, 0, self.STEP),
            Key.right: (0, 0, 0, -self.STEP),
        }
        self.get_logger().info(
            f"""
This node takes keypresses from the keyboard and publishes them 
as Twist messages. This is the incremental mode; every key press 
incrementally increase or decrease the respective dimensional speed.

WARNING: This node will take commands even if your terminal is not in focus!

Controls:

w:+linearX , s:-linearX
q:+linearY , z:-linearY
e:+linearZ , c:-linearZ
a:-yaw , d:+yaw
Any other key to stop
CTRL-C or . to quit

Configuration:

Increment per keypress: {self.STEP} m/s or rad/s
Max Linear SpeedX: +/-{self.LINEAR1_MAX} m/s
Max Linear SpeedY: +/-{self.LINEAR2_MAX} m/s
Max Linear SpeedZ: +/-{self.LINEAR3_MAX} m/s
Max Angular Speed: +/-{self.ANGULAR_MAX} rad/s
"""
        )

    def update_twist(self, key):
        binding = None
        if self._is_special_key(key):
            if key in self.special_keys_bindings:
                binding = self.special_keys_bindings[key]
            else:
                self.write_twist(0.0, 0.0, 0.0, 0.0)
        else:
            if key.char == ".":
                os.kill(os.getpid(), signal.SIGINT)
            if key.char in self.keys_bindings:
                binding = self.keys_bindings[key.char]
            else:
                self.write_twist(0.0, 0.0, 0.0, 0.0)
        if binding is not None:
            new_linear1 = max(
                min(self.LINEAR1_MAX, self.linear1 + binding[0]), -self.LINEAR1_MAX
            )
            new_linear2 = max(
                min(self.LINEAR1_MAX, self.linear2 + binding[1]), -self.LINEAR2_MAX
            )
            new_linear3 = max(
                min(self.LINEAR3_MAX, self.linear3 + binding[2]), -self.LINEAR3_MAX
            )
            new_angular = max(
                min(self.ANGULAR_MAX, self.angular + binding[3]), -self.ANGULAR_MAX
            )
            self.write_twist(new_linear1, new_linear2, new_linear3, new_angular)

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except AttributeError:
            return True


def main():
    try:
        rclpy.init()
        node = IncrementalKeyTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
