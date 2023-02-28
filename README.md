```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/tonynajjar/keyboard_teleop/jammy-humble/ ./" | sudo tee /etc/apt/sources.list.d/tonynajjar_keyboard_teleop.list
echo "yaml https://raw.githubusercontent.com/tonynajjar/keyboard_teleop/jammy-humble/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-tonynajjar_keyboard_teleop.list
```
