# MoveItCpp Overview

## What is MoveItCpp?

`MoveItCpp` is a high-level C++ interface that provides direct in-process access to MoveIt's planning infrastructure without relying on the `move_group` ROS node.

It gives access to:

* planning pipelines
* planning scene
* robot state monitoring
* trajectory execution
* collision environment
* kinematics

It is mainly intended for:

* advanced robotics applications
* industrial systems
* autonomous manipulation
* custom planning architectures
* realtime-sensitive systems

---

# Core Architecture

```text
Application
    ↓
MoveItCpp
    ├── PlanningSceneMonitor
    ├── PlanningComponent
    ├── Planning Pipelines
    ├── Robot Model
    └── Trajectory Execution
```

MoveItCpp internally uses:

* Motion Planning Pipeline
* OMPL / CHOMP / STOMP / Pilz planners

---

# Main Components

## 1. MoveItCpp

Main entry point for MoveIt functionality.

```cpp
auto moveit_cpp =
    std::make_shared<moveit_cpp::MoveItCpp>(node);
```

Responsible for initializing:

* planners
* planning scene
* robot model
* state monitor
* execution interfaces

---

## 2. PlanningComponent

High-level planning interface for a specific planning group.

```cpp
auto planning_component =
    std::make_shared<moveit_cpp::PlanningComponent>(
        "panda_arm",
        moveit_cpp);
```

Used for:

* setting start states
* setting goals
* generating plans

---

## 3. PlanningSceneMonitor

Maintains:

* current robot state
* collision objects
* world geometry
* sensor updates

Provides the collision-checking environment.

---

# Typical Planning Flow

```text
Initialize MoveItCpp
        ↓
Create PlanningComponent
        ↓
Set start state
        ↓
Set goal
        ↓
Call planner
        ↓
Get trajectory
        ↓
Execute or visualize
```

---

# Goal Types in MoveItCpp

## Pose Goal

```cpp
planning_component->setGoal(target_pose);
```

Planner:

1. solves IK
2. generates trajectory

---

## Joint-Space Goal

```cpp
planning_component->setGoal(robot_state);
```

Goal is already a valid joint configuration.

No IK solving required during planning.

---

## Named Goal

```cpp
planning_component->setGoal("ready");
```

Uses predefined SRDF robot states.

---

# Collision Object Example

MoveItCpp allows direct planning scene manipulation.

```cpp
scene->processCollisionObjectMsg(collision_object);
```

Useful for:

* obstacle avoidance
* dynamic environments
* perception-integrated planning

---

# Important Advantages of MoveItCpp

## 1. Direct Access to MoveIt Internals

Provides direct access to:

* planning scene
* robot states
* planners
* collision world

---

## 2. Better Performance

Since everything runs inside the same process:

* lower latency
* no ROS action overhead
* faster planning interaction

---

## 3. Better for Advanced Systems

Useful for:

* custom planners
* parallel planning
* autonomous systems
* industrial robotics software
* multi-threaded applications

---

# MoveItCpp vs MoveGroupInterface

## Main Difference

### MoveGroupInterface

`MoveGroupInterface` communicates with the external `move_group` node using ROS actions/services.

Architecture:

```text
Application
    ↓
MoveGroupInterface
    ↓
move_group node
    ↓
Planning Pipeline
    ↓
Planner
```

---

### MoveItCpp

MoveItCpp directly embeds the planning infrastructure inside the application.

Architecture:

```text
Application
    ↓
MoveItCpp
    ↓
Planning Pipeline
    ↓
Planner
```

---

# Key Differences

| Feature                     | MoveGroupInterface | MoveItCpp     |
| --------------------------- | ------------------ | ------------- |
| Requires `move_group` node  | Yes                | No            |
| Uses ROS actions/services   | Yes                | No            |
| Ease of use                 | Easier             | More advanced |
| Access to planning scene    | Limited            | Full          |
| Access to planners          | Indirect           | Direct        |
| Performance                 | Moderate           | Better        |
| Customization               | Limited            | Extensive     |
| Best for beginners          | Yes                | No            |
| Best for industrial systems | Limited            | Yes           |

---

# Motion Planning Pipeline Relation

MoveItCpp internally uses the Motion Planning Pipeline.

```text
MoveItCpp
    ↓
Planning Pipeline
    ↓
Planner Plugins
```

The planning pipeline is responsible for:

* request processing
* planner execution
* post-processing
* trajectory generation

MoveItCpp is a higher-level application framework built on top of it.

---

# When to Use MoveItCpp

Use MoveItCpp when building:

* autonomous robots
* industrial manipulation systems
* perception + planning systems
* advanced robotics software
* custom planning architectures

Avoid it initially if:

* learning MoveIt basics
* doing simple motion planning
* rapid prototyping

In those cases, `MoveGroupInterface` is simpler.

---

# One-Line Summary

```text
MoveItCpp = Direct in-process MoveIt application framework with advanced planning control.
```
