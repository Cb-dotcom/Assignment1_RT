# Assignment1_RT1: Turtlesim Teleop + Safety Monotoring Node

This project implements a small ROS 2 system with two turtlesim robots:

- a UI / teleop node to control the turtles from the keyboard
- a distance / safety node that monitors them and blocks unsafe commands

Additional part for exam required:
-  Create a custom message for publishing a string, the robot's linear velocity along x, and angular velocity along z. 

-  Use the custom message in your assignment, publishing at each iteration the robot's velocity and the string "velocity"


## 1. How to run

### 1.1 Build the workspace

From your ROS 2 workspace root (ex `ros2_ws`):

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

NB: This has to be done in each new terminal opened.

### 1.2 Start turtlesim + safety node
```
ros2 launch assignment1_rt bringup.launch.py
```
### 1.3 Start the teleop UI
```
ros2 run assignment1_rt turtle_ui_node
```
## 2. Exam implementation
  Added custom message `assignment1_rt/msg/VelocityStatus`:
- `string label`
- `float32 linear_x`
- `float32 angular_z`

`distance_monitor_node` now publishes (50 Hz) the safe velocities with `label="velocity"`:
- `/turtle1/velocity`
- `/turtle2/velocity`

### 2.1 Testing commands:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /turtle1/velocity
ros2 topic echo /turtle2/velocity
```

## 3. Project/Package File Architecture & rqt_graph

```text
src
├── assignment1_rt
│   ├── CMakeLists.txt
│   ├── config
│   │   └── distance_params.yaml
│   ├── include
│   │   └── assignment1_rt
│   │       ├── distance_monitor_node.hpp
│   │       └── turtle_teleop_node.hpp
│   ├── launch
│   │   └── bringup.launch.py
│   ├── msg
│   │   └── VelocityStatus.msg
│   ├── package.xml
│   └── src
│       ├── distance_monitor_node.cpp
│       └── turtle_ui_node.cpp
```

