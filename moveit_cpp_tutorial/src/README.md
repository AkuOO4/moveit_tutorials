# src/moveit_cpp_tutorials.cpp

Entry point for the MoveItCpp demo. Runs five motion planning examples for the Panda arm in sequence, pausing between each for user input via RvizVisualToolsGui.

## Key objects

| Object | Type | Purpose |
|--------|------|---------|
| `moveit_cpp_ptr` | `moveit_cpp::MoveItCpp` | Core interface — wraps planning scene monitor, robot model, and execution |
| `planning_components` | `moveit_cpp::PlanningComponent` | Sets start/goal states and triggers planning for `panda_arm` |
| `visual_tools` | `moveit_visual_tools::MoveItVisualTools` | Publishes markers and trajectory lines to RViz; handles step prompts |

## Node setup

The node (`run_moveit_cpp`) is created with `automatically_declare_parameters_from_overrides(true)` so that MoveIt configuration parameters passed from the launch file (robot description, OMPL settings, etc.) are accepted without explicit declaration.

A `SingleThreadedExecutor` is spun in a detached thread to keep the current state monitor alive while the main thread runs the planning logic sequentially.

## Plans walkthrough

### Plan 1 — Pose goal from current state
```
start : current robot state
goal  : panda_link8 at position (0.28, -0.2, 0.5) in panda_link0 frame
method: planning_components->setGoal(PoseStamped, link_name)
```

### Plan 2 — Custom start state via IK
```
start : IK solution for panda_link8 at (0.55, 0.0, 0.6)
goal  : reuse target_pose1 from Plan 1
method: planning_components->setStartState(RobotState)
```
`start_state.setFromIK(joint_model_group_ptr, pose)` solves IK and stores the joint configuration in `start_state`.

### Plan 3 — Goal state via IK
```
start : reuse start state from Plan 2
goal  : IK solution for panda_link8 at (0.55, -0.05, 0.8)
method: planning_components->setGoal(RobotState)
```

### Plan 4 — Named goal state
```
start : reuse start state from Plan 3
goal  : "ready" named state defined in panda_arm.xacro SRDF
method: planning_components->setGoal("ready")
```

### Plan 5 — Planning around a collision object
```
start : current robot state
goal  : "extended" named state
scene : box (0.1 × 0.4 × 0.1 m) at (0.4, 0.0, 1.0) added via LockedPlanningSceneRW
```
The collision object is added by locking the planning scene with `LockedPlanningSceneRW` and calling `processCollisionObjectMsg`. The planner automatically avoids the box.

## Execution (disabled by default)

Each plan block contains a commented-out execute call:
```cpp
/* bool blocking = true; */
/* moveit_cpp_ptr->execute(plan_solution.trajectory, blocking, CONTROLLERS); */
```
Uncomment these to send trajectories to the `panda_arm_controller`. The `CONTROLLERS` vector must list the controller names that own the joints being moved.

## Visualization pattern

Every successful plan follows this sequence:
```cpp
visual_tools.publishAxisLabeled(...);   // draw start and goal frames
visual_tools.publishTrajectoryLine(...);// draw the planned path
visual_tools.trigger();                 // flush all markers to RViz
```
`trigger()` batches the marker messages; nothing appears in RViz until it is called.
