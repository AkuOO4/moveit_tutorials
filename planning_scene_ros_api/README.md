# Planning Scene ROS API Tutorial

This package demonstrates how to interact with the MoveIt planning scene using the ROS 2 API in C++. It shows how to add, attach, detach, and remove collision objects in the planning scene, and how to use both topic and service interfaces for scene updates.

## What is this?
This tutorial provides a practical example of manipulating the planning scene in MoveIt. It covers:
- Publishing planning scene diffs to add or remove objects
- Attaching and detaching objects to/from the robot
- Using both asynchronous (topic) and synchronous (service) methods for scene updates
- Visualizing each step in RViz with prompts for user interaction

This is useful for understanding how to programmatically manage the robot's environment and attached objects for motion planning and collision checking.

## How to launch
To run the tutorial, use the following command:

```bash
ros2 launch planning_scene_ros_api planning_scene_ros_api_tutorial.launch.py
```

This will start the demo node and allow you to step through the example in RViz.
