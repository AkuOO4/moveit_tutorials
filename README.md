
# MoveIt! Tutorials

This repository provides a collection of MoveIt! tutorials rganized for ROS 2. It is intended for learning and experimenting with motion planning, robot state management, and Move Group interfaces.


## Repository Structure

- **hello_moveit/**: Basic MoveIt! usage and path planning examples
- **motion_planning_pipeline/**: MoveIt! planning pipeline demonstrations (see both `planning_pipeline.cpp` and `planning_api.cpp` for different API usage)
- **move_group_interface/**: Move Group C++ interface usage
- **robot_state/**: Robot state tutorials
- **planning_scene_ros_api/**: Planning scene manipulation and collision object tutorials


## Quickstart

1. Clone this repository into your ROS 2 workspace
2. Install dependencies with rosdep
3. Build using colcon
4. Source the workspace


## Launching Demos

Each package contains its own README with details. Here are some quick launch commands for the main demos:

- Planning Pipeline demo:
	```bash
	ros2 launch motion_planning_pipeline planning_pipeline.launch.py
	```
- Planning API demo:
	```bash
	ros2 launch motion_planning planning_api.launch.py
	```
- Planning Scene ROS API tutorial:
	```bash
	ros2 launch planning_scene_ros_api planning_scene_ros_api_tutorial.launch.py
	```

See the README in each package for more details and usage examples.

## Contributing & License

Contributions are welcome! See individual package LICENSE files for details.

---

For more information, refer to the README files inside each package folder.
