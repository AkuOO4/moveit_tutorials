# move_group_interface

This package shows how to use the Move Group C++ interface for planning and execution with the Panda robot in ROS 2.

## Contents
- Example C++ node: `src/move_group_interface.cpp`
- Launch files: `launch/move_group_interface.launch.py`, `launch/move_group.launch.py`
- RViz configuration: `rviz/move_group.rviz`

## Usage
Launch the Move Group interface demo with:
```bash
ros2 launch move_group_interface move_group_interface.launch.py
```

## Purpose
- Learn Move Group C++ API
- Execute planned trajectories
- Visualize robot motion in RViz
