# MoveIt Motion Planning — API and Pipeline

This document explains the concepts demonstrated in two source files:

- **`planning_api.cpp`** — accesses a planner plugin directly, without any pipeline abstraction
- **`planning_pipeline.cpp`** — uses `PlanningPipeline`, which wraps the planner with pre/post-processing *request adapters* and integrates live world monitoring

Understanding both gives you the full picture of how motion planning is layered inside MoveIt.

---

## Table of Contents

### planning_api.cpp
1. [RobotModelLoader and RobotModel](#1-robotmodelloader-and-robotmodel)
2. [PlanningScene (Local)](#2-planningscene-local)
3. [Plugin-Based Planner Loading](#3-plugin-based-planner-loading)
4. [MotionPlanRequest and Constraints](#4-motionplanrequest-and-constraints)
5. [Pose Goal via Kinematic Constraints](#5-pose-goal-via-kinematic-constraints)
6. [Joint Space Goal](#6-joint-space-goal)
7. [Path Constraints (Orientation)](#7-path-constraints-orientation)

### planning_pipeline.cpp
8. [PlanningSceneMonitor — Live World State](#8-planningscenemonitor--live-world-state)
9. [PlanningPipeline](#9-planningpipeline)
10. [LockedPlanningSceneRO — Thread-Safe Access](#10-lockedplanningscenero--thread-safe-access)
11. [Planning Request Adapters](#11-planning-request-adapters)

### Both Files
12. [How the Two Approaches Compare](#12-how-the-two-approaches-compare)

---

## 1. RobotModelLoader and RobotModel

**What it is:** `RobotModelLoader` reads the URDF and SRDF from the ROS parameter server (`robot_description`) and builds a `RobotModel` — a complete kinematic and geometric description of the robot.

**The key idea:** The `RobotModel` is the foundation of everything in MoveIt. It knows:
- All link names, shapes, and masses
- All joint names, types, and limits
- Which links belong to which planning groups
- The kinematic chains used by IK solvers

Once loaded, this model is shared across all MoveIt components via a `shared_ptr`. Loading it is expensive; you do it once at startup.

**What the URDF provides vs. the SRDF:** The URDF describes the physical robot (geometry, inertia, joints). The SRDF adds semantic information — planning groups, named configurations (like "ready"), allowed collision pairs, and end-effector definitions.

---

## 2. PlanningScene (Local)

**What it is:** A `PlanningScene` is an in-memory snapshot of the world: the robot's current configuration, all known collision objects, and the allowed-collision matrix.

```cpp
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
```

**The key idea:** In `planning_api.cpp`, the planning scene is *local* — it exists only in this process and is manually set to the "ready" named configuration. It does not receive live updates from the robot or sensors.

**Contrast with PlanningSceneMonitor (used in `planning_pipeline.cpp`):** A local scene is appropriate for offline planning or testing. For real robots, you need a monitored scene that stays synchronized with joint state topics and collision object updates.

**Named configurations:** "ready" is a named joint configuration defined in the SRDF. Setting the scene to "ready" means planning starts from a known, sensible arm pose rather than the zero configuration (which may be in self-collision).

---

## 3. Plugin-Based Planner Loading

**What it is:** MoveIt loads planners as ROS *plugins* via `pluginlib`. This means the planner algorithm is not compiled into your node — it is discovered and loaded at runtime.

```cpp
planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
    "moveit_core", "planning_interface::PlannerManager"));
planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_name));
```

**The key idea:** Any planner that implements the `PlannerManager` interface can be dropped in without changing your application code. The planner name is read from a ROS parameter (`ompl.planning_plugins`), so you can switch planners by changing a config file rather than recompiling.

**What plugins are available:** Common choices include OMPL planners (RRT, RRT*, PRM, EST), STOMP, CHOMP, and Pilz Industrial Motion Planner. Each has different strengths regarding planning time, path quality, and constraint handling.

**Why `createUnmanagedInstance`:** This gives you a raw pointer with manual lifetime management, unlike `createInstance` which returns a `shared_ptr` but requires the loader to stay alive. The code stores both the loader and the instance to keep them in scope.

---

## 4. MotionPlanRequest and Constraints

**What it is:** `MotionPlanRequest` is the message that fully describes a planning problem. It contains:
- `group_name` — which planning group to plan for
- `goal_constraints` — what the robot must achieve
- `path_constraints` — what the robot must maintain throughout the motion
- `workspace_parameters` — bounding box for end-effector sampling
- `allowed_planning_time` — timeout for the planner

**The key idea:** MoveIt expresses *all* goals — pose goals, joint goals, and constraints — as instances of the `Constraints` message type. The `kinematic_constraints` helper package converts familiar input (a pose, a robot state) into this unified format.

**Why tolerance vectors have three elements:** Position tolerance is `[x_tol, y_tol, z_tol]` and angle tolerance is `[roll_tol, pitch_tol, yaw_tol]`. This lets you specify asymmetric tolerances, though in practice equal values are most common.

---

## 5. Pose Goal via Kinematic Constraints

**What it is:** A pose goal expressed as kinematic constraints, targeting a specific link's position and orientation.

```cpp
moveit_msgs::msg::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);
```

**The key idea:** The planner samples random configurations and checks whether `panda_link8` ends up within the position and orientation tolerance of the target. Tolerances are not a fallback — they define the goal *region*. A tight tolerance (0.01 m, 0.01 rad) is precise but may require more planning time.

**Why `panda_link8` and not the end-effector?** `panda_link8` is the last link in the kinematic chain for the `panda_arm` group. In practice it sits just behind the flange, so tools attached to it inherit its pose.

---

## 6. Joint Space Goal

**What it is:** A goal that specifies exact target angles for each joint, rather than an end-effector pose.

```cpp
moveit::core::RobotState goal_state(robot_model);
std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::msg::Constraints joint_goal =
    kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
```

**The key idea:** Joint goals are unambiguous — there is exactly one arm posture that satisfies them (unlike pose goals, which may have multiple IK solutions). They are ideal for moving to pre-defined configurations like home positions.

**No IK needed:** Because you have specified every joint directly, the planner does not need to call an IK solver. It still needs to find a collision-free path from the current state to the goal, but the goal itself is already in joint space.

---

## 7. Path Constraints (Orientation)

**What it is:** A constraint that must be satisfied at every point along the trajectory, not just at the goal.

```cpp
geometry_msgs::msg::QuaternionStamped quaternion;
quaternion.header.frame_id = "panda_link0";
req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);
```

**The key idea:** Constraining the end-effector orientation to stay near upright (identity quaternion) throughout the motion means the planner must find a path where the wrist never tilts beyond the tolerance. This is the classic "carry a cup of water without spilling" requirement.

**Why workspace bounds are required here:** When path constraints are active, the planner samples in Cartesian space — it samples end-effector positions within the workspace volume and uses IK to convert them to joint configurations. Without a workspace bounding box, the sampler has no bounds to work within. The `±2.0 m` cube set here easily contains the Panda's entire reachable space.

**Performance impact:** With path constraints, most random configurations are invalid because they violate the constraint. Planning takes longer. The code uses a 10-second timeout for this reason (vs. 5 seconds without constraints).

---

## 8. PlanningSceneMonitor — Live World State

**What it is:** `PlanningSceneMonitor` wraps a `PlanningScene` and keeps it synchronized with the live robot and world by subscribing to ROS topics.

```cpp
planning_scene_monitor::PlanningSceneMonitorPtr psm(
    new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));
psm->startSceneMonitor();         // subscribes to /planning_scene diffs
psm->startWorldGeometryMonitor(); // subscribes to collision object updates
psm->startStateMonitor();         // subscribes to /joint_states
```

**The key idea:** For real robots, the planning scene must reflect the actual world. `startStateMonitor()` ensures the robot's joint positions stay current. `startWorldGeometryMonitor()` picks up collision objects published by perception pipelines. `startSceneMonitor()` receives full and diff scene updates from other MoveIt components.

**Why this matters for planning:** If the scene monitor is not running, the planner uses a stale snapshot of the world. A robot that has moved from its last known position, or an obstacle that appeared after startup, would be invisible to the planner — leading to collisions.

---

## 9. PlanningPipeline

**What it is:** `PlanningPipeline` wraps a planner plugin and adds a chain of *request adapters* that pre- and post-process every planning request.

```cpp
planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));
planning_pipeline->generatePlan(lscene, req, res);
```

**The key idea:** The pipeline is not just "call the planner." Before planning, adapters can:
- Fix invalid start states (e.g., clamp joints that are slightly out of bounds)
- Add default workspace bounds

After planning, adapters can:
- Add time parameterization to the trajectory (computing velocities and accelerations)
- Smooth the path

This is why `PlanningPipeline` is what `MoveGroupInterface` uses internally — the adapters ensure the output trajectory is always executable, not just a sequence of waypoints.

**Naming the pipeline:** `"ompl"` selects the OMPL configuration block from the `ompl_planning.yaml` configuration file, which specifies which planner to use and its parameters.

---

## 10. LockedPlanningSceneRO — Thread-Safe Access

**What it is:** A RAII read lock on the planning scene monitor's internal scene.

```cpp
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
  planning_pipeline->generatePlan(lscene, req, res);
}
```

**The key idea:** The planning scene monitor updates its internal scene from multiple ROS subscriber callbacks running in the executor thread. If the planner reads the scene at the same moment a callback is writing to it, you get a data race. `LockedPlanningSceneRO` acquires a shared (read) lock for the duration of planning, blocking any world updates until planning finishes.

**Why the braces matter:** The lock is released when `lscene` goes out of scope — the closing `}` is the unlock. This keeps the lock held for only as long as needed, allowing the scene monitor to resume updating once planning is done.

---

## 11. Planning Request Adapters

**What it is:** A practical demonstration of why adapters matter. The code intentionally sets a joint slightly beyond its lower limit:

```cpp
tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
robot_state->setJointPositions(joint_model, tmp_values);
```

then sends a planning request starting from this invalid state. The pipeline still succeeds, because the `FixStartStateBounds` adapter silently clamps the joint back within limits before passing the request to the planner.

**The key idea:** In practice, floating-point noise, sensor imprecision, or controller overshoot can leave the robot marginally outside its joint limits. Without adapters, every planning request starting from such a state would fail. Adapters make the system robust to these small real-world discrepancies.

**Built-in adapters in the OMPL pipeline:**

| Adapter | What it does |
|---|---|
| `FixStartStateBounds` | Clamps joints that are slightly out of bounds |
| `FixStartStateCollision` | Pushes start state out of collision |
| `AddTimeParameterization` | Computes velocities and accelerations for the raw waypoint path |
| `ResolveConstraintFrames` | Resolves constraint frames to the model frame |

---

## 12. How the Two Approaches Compare

| Aspect | `planning_api.cpp` | `planning_pipeline.cpp` |
|---|---|---|
| Scene | Local, static | Live-monitored via PSM |
| Planner access | Direct plugin instance | Pipeline (planner + adapters) |
| Request adapters | None | Full adapter chain |
| Thread safety | Not needed (single-threaded scene) | LockedPlanningSceneRO required |
| Time parameterization | Manual | Automatic (via adapter) |
| Use case | Testing, offline planning, custom pipelines | Production, real robots |

**In short:** `planning_api.cpp` shows you what MoveIt does at the lowest level. `planning_pipeline.cpp` shows you how MoveIt is actually used in practice, with all the robustness layers enabled.
