# teleop_twist_keyboard
Keyboard Teleop for ROS2 with two modes "incremental" and "hold"

# Installation

- Clone the repo: ```git clone https://github.com/tonynajjar/teleop_twist_keyboard/```
- Install dependencies: ```rosdep install --from-paths src --ignore-src -r```

# Launch

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard_hold 
```
or

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard_incremental
```

To run with custom config: 

```
ros2 run teleop_twist_keyboard key_teleop_incremental --ros-args --params-file <path-to-your-config-file>
```
Configuration examples can be found in teleop_twist_keyboard/config/


# teleop_twist_keyboard_incremental Usage

```
This node takes keypresses from the keyboard and publishes them 
as Twist messages. This is the incremental mode; every key press 
incrementally increase or decrease the respective dimensional speed.

WARNING: This node will take commands even if your terminal is not in focus!

Controls:

WASD or Arrows to increase/decrease speeds
Any other key to stop
CTRL-C or q to quit
```

# teleop_twist_keyboard_hold Usage

```
This node takes keypresses from the keyboard and publishes them 
as Twist messages. This is the holding mode; your keypress will
set the maximum configured speeds, at release all speeds are reset

WARNING: This node will take commands even if your terminal is not in focus!

Controls:

WASD or Arrows to move
Any other key to stop
CTRL-C or q to quit
```
