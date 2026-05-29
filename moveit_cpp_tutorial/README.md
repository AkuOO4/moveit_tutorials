# moveit_cpp_tutorial

A ROS 2 package demonstrating motion planning for the Panda robot arm using the MoveItCpp API. The demo runs five sequential planning examples, each visualized step-by-step in RViz via the RvizVisualToolsGui.

## Prerequisites

- ROS 2 Jazzy
- MoveIt 2
- `panda_moveit_config` package (from `moveit_resources`)
- `moveit_visual_tools`

## Build

From the workspace root:

```bash
cd ~/Robotics/tutorial_ws/moveit_ws
colcon build --packages-select moveit_cpp_tutorial
source install/setup.bash
```

To build along with dependencies:

```bash
colcon build --packages-up-to moveit_cpp_tutorial
source install/setup.bash
```

## Run

```bash
ros2 launch moveit_cpp_tutorial moveit_cpp_tutorial.launch.py
```

This starts:
- RViz with the pre-configured layout
- The `moveit_cpp_tutorials` demo node
- `robot_state_publisher`
- A static TF publisher (`world` → `panda_link0`)
- `ros2_control_node` with the FakeSystem hardware interface
- Controller spawners for `panda_arm_controller`, `panda_hand_controller`, and `joint_state_broadcaster`

## Stepping through the demo

The demo pauses at each step and waits for user input through the **RvizVisualToolsGui** panel in RViz.

1. Open RViz — the `RvizVisualToolsGui` panel should be visible in the bottom-left.
2. Click **Next** to advance through each plan.

The five plans demonstrated are:

| Step | Description |
|------|-------------|
| Plan 1 | Pose goal from current state using `geometry_msgs::PoseStamped` |
| Plan 2 | Custom start state set via IK from a `geometry_msgs::Pose` |
| Plan 3 | Goal state set via IK using `moveit::core::RobotState` |
| Plan 4 | Goal set by named state (`"ready"`) from the SRDF |
| Plan 5 | Planning around a box collision object added to the scene |

> **Note:** Trajectory execution is disabled by default. To enable it, uncomment the `moveit_cpp_ptr->execute(...)` lines in `src/moveit_cpp_tutorials.cpp` and rebuild.

## Package structure

```
moveit_cpp_tutorial/
├── launch/
│   └── moveit_cpp_tutorial.launch.py   # Launches all nodes
├── rviz/
│   └── moveit_cpp_tutorial.rviz        # Pre-configured RViz layout
├── src/
│   └── moveit_cpp_tutorials.cpp        # Demo source code
├── CMakeLists.txt
└── package.xml
```
