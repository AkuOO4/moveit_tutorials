# Hello MoveIt — Getting Started

This document explains the concepts demonstrated in `hello_moveit.cpp`, the simplest possible MoveIt 2 program. It is intentionally minimal: one goal, one plan, one execution.

---

## Table of Contents

1. [MoveGroupInterface — Your Entry Point](#1-movegroupinterface--your-entry-point)
2. [Planning Groups](#2-planning-groups)
3. [Setting a Pose Target](#3-setting-a-pose-target)
4. [Planning](#4-planning)
5. [Executing the Plan](#5-executing-the-plan)
6. [ROS 2 Node Setup](#6-ros-2-node-setup)

---

## 1. MoveGroupInterface — Your Entry Point

**What it is:** `MoveGroupInterface` is the primary high-level API for controlling a robot through MoveIt. It hides the complexity of communicating with the `move_group` node, running planners, monitoring robot state, and sending trajectories to controllers.

**The key idea:** Rather than talking directly to joint controllers, IK solvers, or collision checkers, you talk to a single object that coordinates everything behind the scenes. The `move_group` node does the heavy lifting; `MoveGroupInterface` is your handle into it.

**Why a separate `move_group` node?** MoveIt's architecture separates the planning server (`move_group`) from your application node. This allows multiple client programs to share the same robot model, planning scene, and controller interface without each one loading those resources independently.

---

## 2. Planning Groups

**What it is:** A robot is divided into named subsets of joints called *planning groups* (or *joint model groups*). Here, `"panda_arm"` refers to the seven revolute joints of the Franka Panda arm, excluding the gripper.

**The key idea:** You plan and move one group at a time. The arm has its own planner, IK solver, and controller. The hand (gripper) is a separate group with its own interface. This separation keeps planning tractable — planning all joints simultaneously would be far more expensive and usually unnecessary.

**Where groups are defined:** Planning groups come from the robot's `srdf` (Semantic Robot Description Format) file, part of the `moveit_config` package. MoveIt reads this file at startup.

---

## 3. Setting a Pose Target

**What it is:** A *pose target* is the desired 6-DOF pose (position + orientation) of the end-effector in the world frame.

```cpp
geometry_msgs::msg::Pose msg;
msg.orientation.w = 1.0;   // identity quaternion — no rotation
msg.position.x = 0.28;
msg.position.y = -0.2;
msg.position.z = 0.5;
move_group_interface.setPoseTarget(target_pose);
```

**The key idea:** You specify *where* the tool tip should end up — not *how* the joints should move to get there. MoveIt internally calls an inverse kinematics (IK) solver to find joint configurations that produce this end-effector pose, then plans a collision-free path to one of them.

**Orientation representation:** The `orientation.w = 1.0` (with x, y, z all zero) is the identity quaternion — meaning the end-effector keeps its default orientation relative to the base frame. Quaternions avoid the gimbal lock problems of Euler angles and are the standard representation in ROS.

**When there is no solution:** IK can fail if the target is out of reach, in collision, or in a kinematically singular region (e.g., arm fully extended). The planner will then report failure without attempting to move.

---

## 4. Planning

**What it is:** Planning computes a collision-free trajectory from the current robot state to the goal state, without physically moving the robot.

```cpp
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();
```

**The key idea:** Planning and execution are deliberately separated. You always check whether planning succeeded before attempting to execute. Sending an invalid trajectory to a real robot can cause hardware damage, so this gate is important.

**What the plan contains:** The `Plan` object stores:
- `start_state` — the robot state at planning time
- `trajectory` — a time-parameterized sequence of joint positions, velocities, and accelerations

**Non-determinism:** Motion planners (especially sampling-based ones like OMPL's RRT variants) are probabilistic. The same goal can produce different trajectories on different runs. This is normal and expected.

---

## 5. Executing the Plan

**What it is:** Execution sends the computed trajectory to the robot's joint controllers, which physically move the robot along the planned path.

```cpp
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
```

**The key idea:** `execute()` is a blocking call — it waits until the trajectory is complete (or fails). For real robots this means waiting for the hardware to finish moving. For simulation it waits for the simulated controller.

**Planning vs. moving:** `MoveGroupInterface` also offers a `move()` function that plans and executes in one call. This tutorial separates them explicitly to make the two phases clear. In production code, you typically want the separation so you can inspect, log, or conditionally execute the plan.

---

## 6. ROS 2 Node Setup

**What it is:** The boilerplate that initializes ROS 2 and gives MoveIt a communication handle.

```cpp
rclcpp::init(argc, argv);
auto const node = std::make_shared<rclcpp::Node>(
  "hello_moveit",
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
```

**Why `automatically_declare_parameters_from_overrides`:** MoveIt passes many configuration parameters (planner settings, robot description, controller names) through the ROS parameter system. This option allows parameters set externally — e.g., from a launch file — to be accepted without needing explicit `declare_parameter()` calls in the node.

**No executor in this example:** Unlike more complex tutorials, `hello_moveit` does not spin an executor in a separate thread. This works for a simple one-shot program but would cause problems in long-running nodes that need to handle callbacks concurrently (e.g., for state monitoring or planning scene updates).

---

## Quick Reference

| Concept | API Call | Purpose |
|---|---|---|
| Connect to move_group | `MoveGroupInterface(node, "panda_arm")` | Entry point for all planning |
| Set end-effector target | `setPoseTarget(pose)` | Specify where to move |
| Compute trajectory | `plan(my_plan)` | Find collision-free path |
| Move the robot | `execute(my_plan)` | Send trajectory to controllers |
