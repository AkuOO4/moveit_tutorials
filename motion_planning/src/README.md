# Motion Planning Demos - Code Overview

This README provides a brief explanation of the two main demo files in this package:

- `planning_pipeline.cpp`
- `planning_api.cpp`

Both files demonstrate different ways to use MoveIt for motion planning with a Panda robot arm in ROS 2.

---

## planning_pipeline.cpp

This file demonstrates how to use the MoveIt Planning Pipeline directly for motion planning.

**Key Steps:**
1. **Initialization:**
   - Sets up the ROS 2 node and executor.
   - Loads the robot model and creates a `PlanningSceneMonitor` to keep the planning scene updated.
2. **Planning Pipeline Setup:**
   - Initializes the `PlanningPipeline` object, which manages the planning plugins and request adapters.
3. **Visualization:**
   - Uses `MoveItVisualTools` for RViz visualization and user prompts.
4. **Pose Goal Planning:**
   - Creates a pose goal for the end-effector and sends a planning request.
   - Visualizes the resulting trajectory in RViz.
5. **Joint Space Goal Planning:**
   - Sets a goal in joint space and plans a trajectory.
   - Visualizes the new trajectory.
6. **Planning Request Adapter Example:**
   - Demonstrates how planning request adapters can handle invalid start states (e.g., joint limits violations).
   - Visualizes the result.

---

## planning_api.cpp

This file demonstrates how to use the MoveIt Planning API and plugin system for motion planning.

**Key Steps:**
1. **Initialization:**
   - Sets up the ROS 2 node and executor.
   - Loads the robot model and creates a `PlanningScene`.
2. **Planner Plugin Loading:**
   - Dynamically loads a planner plugin using ROS pluginlib.
   - Initializes the planner with the robot model and node.
3. **Visualization:**
   - Uses `MoveItVisualTools` for RViz visualization and user prompts.
4. **Pose Goal Planning:**
   - Creates a pose goal and sends a planning request to the planner plugin.
   - Visualizes the resulting trajectory.
5. **Joint Space Goal Planning:**
   - Sets a goal in joint space and plans a trajectory.
   - Visualizes the new trajectory.
6. **Path Constraints Example:**
   - Demonstrates planning with path/orientation constraints.
   - Visualizes the result.

---

## Summary
- Both files show how to set up and use MoveIt for motion planning, but with different APIs.
- Visualization and user interaction are handled via RViz and `MoveItVisualTools`.
- The demos cover pose goals, joint space goals, and planning with constraints.

For more details, refer to the code comments in each file.
