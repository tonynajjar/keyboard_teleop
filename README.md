# keyboard_teleop
Keyboard Teleop for ROS2 with two modes "incremental" and "hold"

# Installation

- Clone the repo: ```git clone https://github.com/tonynajjar/keyboard_teleop```
- Install dependencies: ```rosdep install --from-paths src --ignore-src -r```

# Launch

```
ros2 run keyboard_teleop keyboard_teleop_hold 
```
or

```
ros2 run keyboard_teleop keyboard_teleop_incremental
```

To run with custom config: 

```
ros2 run keyboard_teleop key_teleop_incremental --ros-args --params-file <path-to-your-config-file>
```
A configuration example can be found in keyboard_teleop/config/


# keyboard_teleop_incremental Usage

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

# keyboard_teleop_hold Usage

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
