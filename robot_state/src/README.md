# MoveIt Robot State — Kinematics Fundamentals

This document explains the concepts demonstrated in `robot_state_tutorial.cpp`. This tutorial is the foundation for understanding how MoveIt represents a robot's configuration and performs kinematic computations — forward kinematics, inverse kinematics, and Jacobian calculation.

---

## Table of Contents

1. [RobotModelLoader and RobotModel](#1-robotmodelloader-and-robotmodel)
2. [RobotState](#2-robotstate)
3. [JointModelGroup](#3-jointmodelgroup)
4. [Reading and Writing Joint Values](#4-reading-and-writing-joint-values)
5. [Joint Limits and enforceBounds](#5-joint-limits-and-enforcebounds)
6. [Forward Kinematics (FK)](#6-forward-kinematics-fk)
7. [Inverse Kinematics (IK)](#7-inverse-kinematics-ik)
8. [The Jacobian](#8-the-jacobian)
9. [How These Connect to Motion Planning](#9-how-these-connect-to-motion-planning)

---

## 1. RobotModelLoader and RobotModel

**What it is:** `RobotModelLoader` reads the robot description from the ROS parameter server and builds an immutable `RobotModel` — the kinematic and geometric definition of the robot.

```cpp
robot_model_loader::RobotModelLoader robot_model_loader(node);
const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());
```

**The key idea:** The `RobotModel` is the "static" description — it does not change during runtime. It encodes:
- All links (geometry, inertia, visual and collision meshes)
- All joints (type, axis, limits, mimic relationships)
- All planning groups defined in the SRDF
- The kinematic chains and IK solver plugins for each group

**Model frame:** The frame printed by `getModelFrame()` is the root frame of the robot — typically `panda_link0` for the Panda. All link transforms are expressed relative to this frame.

**Cost of loading:** Building the model parses the URDF and SRDF, loads plugin descriptors, and constructs kinematic chains. It is expensive. Load once at startup and share the `shared_ptr` everywhere.

---

## 2. RobotState

**What it is:** A `RobotState` is a mutable snapshot of the robot's complete configuration: position, velocity, and acceleration for every joint.

```cpp
moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
robot_state->setToDefaultValues();
```

**The key idea:** `RobotModel` is the *schema* (what joints exist, what their limits are). `RobotState` is an *instance* of that schema with specific values filled in. You can create as many `RobotState` objects as you want from the same `RobotModel` — they are cheap and independent.

**`setToDefaultValues()`:** Puts every joint at its zero position (or named default position if one is defined). For many robots this is a flat/straight configuration. For the Panda, zero values may put the arm in self-collision, which is why `"ready"` is often used instead.

**What RobotState is used for:**
- Storing the start state for planning
- Computing FK and IK
- Checking self-collision and joint limit validity
- Interpolating between configurations

---

## 3. JointModelGroup

**What it is:** A `JointModelGroup` represents a subset of the robot's joints that are planned and moved together — a planning group.

```cpp
const moveit::core::JointModelGroup* joint_model_group =
    kinematic_model->getJointModelGroup("panda_arm");
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
```

**The key idea:** The full robot may have many joints (arm, gripper, torso, head). You rarely plan all of them simultaneously. A `JointModelGroup` scopes operations to just the relevant joints, making IK, FK, and collision checking faster and more focused.

**Why a raw pointer:** `getJointModelGroup()` returns a non-owning raw pointer to an object owned by the `RobotModel`. As long as the `RobotModel` is alive, this pointer is valid. Never delete it manually.

**What the group provides:**
- List of joint names in order
- Mapping from variable names to indices
- The kinematic chain (root link to tip link)
- Which IK solver plugin handles this group

---

## 4. Reading and Writing Joint Values

**What it is:** Getting and setting the position of every joint in a group.

```cpp
// Read
std::vector<double> joint_values;
robot_state->copyJointGroupPositions(joint_model_group, joint_values);

// Write
joint_values[0] = 5.57;
robot_state->setJointGroupPositions(joint_model_group, joint_values);
```

**The key idea:** Joint values are a flat `std::vector<double>` in the same order as `joint_model_group->getVariableNames()`. The index mapping is deterministic — joint 0 always corresponds to the first variable name in the group.

**`copy` vs. `get`:** `copyJointGroupPositions` copies values into your vector (safe, independent). There are also `get` variants that return references or pointers into the state's internal storage — faster but only valid while the state hasn't been modified.

**Multi-DOF joints:** Some joints (like a floating base) have multiple variables (x, y, z, qx, qy, qz, qw). `getVariableNames()` returns all variables, which is why the count may exceed the number of joints.

---

## 5. Joint Limits and enforceBounds

**What it is:** Every joint has a valid range defined in the URDF. `satisfiesBounds()` checks whether the current state respects those limits; `enforceBounds()` clamps any out-of-range joints back to their nearest limit.

```cpp
joint_values[0] = 5.57;  // intentionally beyond the joint's upper limit
robot_state->setJointGroupPositions(joint_model_group, joint_values);

RCLCPP_INFO_STREAM(LOGGER, "Current state is "
    << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

robot_state->enforceBounds();
RCLCPP_INFO_STREAM(LOGGER, "Current state is "
    << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
```

**The key idea:** `setJointGroupPositions()` does not silently clamp values — it lets you store out-of-bounds positions. This is intentional: you might want to detect that a commanded value was out of range before deciding what to do. `enforceBounds()` is the explicit correction step.

**Why this matters in practice:**
- Sensor noise or floating-point math can put joints marginally out of bounds
- Planning with an out-of-bounds start state causes planners to fail or produce poor paths
- The `FixStartStateBounds` planning request adapter (used in `planning_pipeline.cpp`) automates this correction

**Bounds checking granularity:** `satisfiesBounds()` has an overload that accepts a tolerance, so you can distinguish "exactly in bounds" from "within 1e-6 radians of the limit."

---

## 6. Forward Kinematics (FK)

**What it is:** Given joint angles, compute the resulting position and orientation of a link in the world (or model) frame.

```cpp
robot_state->setToRandomPositions(joint_model_group);
const Eigen::Isometry3d& end_effector_state =
    robot_state->getGlobalLinkTransform("panda_link8");

RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");
```

**The key idea:** FK is a deterministic, closed-form computation. Given joint angles, there is exactly one answer for where the link ends up — no iteration, no ambiguity. It is cheap and always succeeds.

**`Eigen::Isometry3d`:** This is a 4×4 homogeneous transformation matrix representing both translation and rotation. The `.translation()` method returns a 3D vector; `.rotation()` returns a 3×3 rotation matrix. This is MoveIt's standard representation for poses throughout the library.

**`setToRandomPositions()`:** Samples each joint uniformly within its valid range, producing a valid (but random) robot configuration. Useful for testing FK/IK without hand-crafting joint values.

**`getGlobalLinkTransform()`:** Returns the transform of the named link relative to the model root frame. "Global" here means "in the model frame," not "in the world frame" — unless the model frame and world frame coincide (which they do when the robot base is fixed).

---

## 7. Inverse Kinematics (IK)

**What it is:** The reverse of FK — given a desired end-effector pose, find joint angles that achieve it.

```cpp
double timeout = 0.1;
bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);

if (found_ik)
{
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  // joint_values now holds the IK solution
}
```

**The key idea:** IK is generally harder than FK. For a 7-DOF arm like the Panda, there are infinitely many joint configurations that place the end-effector at the same pose (the arm is *redundant*). IK solvers return one solution, but there may be many valid ones.

**Why IK can fail:**
- The target pose is outside the robot's reachable workspace
- The target pose is inside the robot's body (self-collision)
- The robot is near a kinematic singularity (determinant of Jacobian ≈ 0)
- The timeout expires before the numerical solver converges

**Timeout parameter:** IK for redundant robots is typically solved numerically (iterative gradient descent). The timeout (0.1 s here) caps how long the solver tries. Tighter configurations or poor initial guesses may need longer timeouts.

**IK as a planning building block:** Every time the `MoveGroupInterface` plans to a pose goal, it calls IK internally to convert the pose into joint configurations that it can then plan a path to. Understanding IK helps diagnose why pose-goal planning sometimes fails.

---

## 8. The Jacobian

**What it is:** The Jacobian matrix `J` relates joint velocities to end-effector velocities: `ẋ = J(q) q̇`. It is a 6×N matrix (6 end-effector DOF × N joint DOF).

```cpp
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
Eigen::MatrixXd jacobian;
robot_state->getJacobian(joint_model_group,
    robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
    reference_point_position,
    jacobian);
RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
```

**The key idea:** The Jacobian is configuration-dependent — its value changes with every different set of joint angles. It is the core tool for:
- **Singularity detection:** When `det(J) ≈ 0`, the robot loses the ability to move in certain directions
- **Velocity-level control:** Given a desired end-effector velocity, compute the required joint velocities via `q̇ = J⁺ ẋ` (pseudoinverse)
- **Differential IK:** Iteratively solve IK by taking small steps guided by the Jacobian

**Reference point:** The `reference_point_position` is an offset from the link's origin at which the end-effector velocity is computed. `(0,0,0)` means the Jacobian is computed at the link's origin itself.

**Rows of the Jacobian:** The 6 rows correspond to `[vx, vy, vz, ωx, ωy, ωz]` — three translational and three rotational velocity components of the end-effector.

**Singularities in practice:** When the arm is fully extended (elbow locked out) or when two joint axes align, the Jacobian becomes rank-deficient. The pseudoinverse blows up and velocity commands produce enormous joint velocities. Motion planners and controllers must avoid or handle singularities carefully.

---

## 9. How These Connect to Motion Planning

The concepts in this tutorial underpin everything that happens when you call `move_group.plan()`:

| Planning step | Underlying concept |
|---|---|
| "Is the start state valid?" | `satisfiesBounds()` + self-collision check |
| "Can I reach this pose?" | `setFromIK()` — pose goal requires a valid IK solution |
| "What's the arm doing now?" | `getGlobalLinkTransform()` — FK from current joint state |
| "Is this configuration safe?" | `RobotState` collision check against `PlanningScene` |
| "How do I move smoothly to the goal?" | Jacobian used by CHOMP/STOMP and velocity controllers |

Understanding `RobotState` and kinematics at this level lets you debug planning failures, write custom constraints, or implement your own controllers outside the MoveIt pipeline.

---

## Quick Reference

| Concept | API | Key Point |
|---|---|---|
| Robot description | `RobotModelLoader` → `RobotModel` | Load once; share via `shared_ptr` |
| Configuration snapshot | `RobotState` | Mutable; many per `RobotModel` |
| Joint subset | `JointModelGroup` | Raw pointer; owned by `RobotModel` |
| Read joint angles | `copyJointGroupPositions()` | Ordered by `getVariableNames()` |
| Write joint angles | `setJointGroupPositions()` | Does not enforce limits automatically |
| Validate limits | `satisfiesBounds()` / `enforceBounds()` | Check before planning |
| Pose from joints | `getGlobalLinkTransform()` | Always succeeds, deterministic |
| Joints from pose | `setFromIK()` | May fail; redundant robots have many solutions |
| Velocity mapping | `getJacobian()` | Configuration-dependent; zero determinant = singularity |
