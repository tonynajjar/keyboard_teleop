import os
import signal

import rclpy
from pynput.keyboard import Key, Listener

from keyboard_teleop.teleop import Teleop


class HoldKeyTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.key_listener = Listener(
            on_press=self.update_twist,
            on_release=self.on_release,
        )
        self.key_listener.start()
        self.keys_bindings = {
            "w": (self.LINEAR_MAX, 0.0),
            "s": (-self.LINEAR_MAX, 0.0),
            "a": (0.0, self.ANGULAR_MAX),
            "d": (0.0, -self.ANGULAR_MAX),
        }
        self.special_keys_bindings = {
            Key.up: (self.LINEAR_MAX, 0.0),
            Key.down: (-self.LINEAR_MAX, 0.0),
            Key.left: (0.0, self.ANGULAR_MAX),
            Key.right: (0.0, -self.ANGULAR_MAX),
        }
        self.get_logger().info(
            f"""
This node takes keypresses from the keyboard and publishes them 
as Twist messages. This is the holding mode; your keypress will
set the maximum configured speeds, at release all speeds are reset

WARNING: This node will take commands even if your terminal is not in focus!

Controls:

WASD or Arrows to move
Any other key to stop
CTRL-C or q to quit

Configuration:

Max Linear Speed: +/-{self.LINEAR_MAX} m/s
Max Angular Speed: +/-{self.ANGULAR_MAX} rad/s
"""
        )

    def on_release(self, key):
        if self._is_special_key(key):

            if key in self.special_keys_bindings:
                if key == Key.up or key == Key.down:
                    self.write_twist(linear=0.0)
                elif key == Key.left or key == Key.right:
                    self.write_twist(angular=0.0)
        else:
            key = key.char
            if key in self.keys_bindings:
                if key == "w" or key == "s":
                    self.write_twist(linear=0.0)
                elif key == "a" or key == "d":
                    self.write_twist(angular=0.0)

    def update_twist(self, key):
        binding = None
        if self._is_special_key(key):
            if key in self.special_keys_bindings:
                binding = self.special_keys_bindings[key]
            else:
                self.write_twist(0.0, 0.0)
        else:
            if key.char == "q":
                os.kill(os.getpid(), signal.SIGINT)
            if key.char in self.keys_bindings:
                binding = self.keys_bindings[key.char]
            else:
                self.write_twist(0.0, 0.0)
        if binding is not None:
            new_linear = binding[0]
            new_angular = binding[1]
            self.write_twist(new_linear, new_angular)

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except AttributeError:
            return True


def main():
    try:
        rclpy.init()
        node = HoldKeyTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
