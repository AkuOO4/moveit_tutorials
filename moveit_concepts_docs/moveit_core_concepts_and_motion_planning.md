# MoveIt Core Concepts and Motion Planning 

## 1. RobotModel and RobotState

### RobotModel

`RobotModel` represents the static structure of the robot.

It contains:

- Links
- Joints
- Joint limits
- Kinematic tree
- Planning groups
- Collision geometry
- URDF and SRDF information

Think of it as:

> What the robot physically is.

---

### RobotState

`RobotState` represents the current runtime configuration of the robot.

It stores:

- Joint positions
- Velocities
- Accelerations

It is used for:

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Jacobian computation
- Collision checking
- Constraint checking

Think of it as:

> What the robot is currently doing.

---

### JointModelGroup

A `JointModelGroup` represents a planning group defined in SRDF.

Examples:

- `panda_arm`
- `manipulator`
- `gripper`

MoveIt operations are usually performed on groups instead of the entire robot.

---

## 2. Joint Limits

Joint limits define the allowed motion range of robot joints.

Types of limits:

### Position Limits

Maximum joint range.

Example:

```yaml
joint1:
  min_position: -2.89
  max_position: 2.89
```

---

### Velocity Limits

Maximum joint speed.

Example:

```text
max_velocity = 2 rad/s
```

---

### Acceleration Limits

Maximum joint acceleration.

Example:

```text
max_acceleration = 5 rad/s²
```

---

### Important Functions

Check limits:

```cpp
robot_state->satisfiesBounds();
```

Enforce limits:

```cpp
robot_state->enforceBounds();
```

---

## 3. Forward Kinematics (FK)

Forward Kinematics computes the end-effector pose from joint values.

Conceptually:

```text
joint angles -> end-effector pose
```

Equation:

```text
T = f(q)
```

Where:

- `q` = joint values
- `T` = end-effector transform

---

### FK Example

```cpp
robot_state->getGlobalLinkTransform("panda_link8");
```

Returns:

- Position
- Orientation
- 4x4 transformation matrix

---

## 4. Inverse Kinematics (IK)

Inverse Kinematics computes joint values required to reach a desired pose.

Conceptually:

```text
end-effector pose -> joint angles
```

Equation:

```text
q = f^-1(T)
```

---

### IK Example

```cpp
robot_state->setFromIK(...);
```

IK may fail due to:

- Unreachable target
- Joint limits
- Singularities
- Timeout
- Invalid constraints

---

## 5. Jacobian

### What Is the Jacobian?

The Jacobian describes how joint motion affects end-effector motion.

Simplest interpretation:

> It tells how each joint contributes to tool movement.

---

### Jacobian Equation

```text
x_dot = J(q) * q_dot
```

Where:

- `q_dot` = joint velocities
- `J(q)` = Jacobian matrix
- `x_dot` = end-effector velocity

---

### Important Intuition

Each column of the Jacobian represents:

> What happens to the end effector if only that joint moves.

---

### Jacobian Dimensions

For a 7-DOF Panda robot:

```text
6 x 7
```

Rows:

```text
vx
vy
vz
wx
wy
wz
```

Columns:

```text
joint1 ... joint7
```

---

### Jacobian Usage

Used for:

- Cartesian control
- Differential IK
- Force control
- Velocity control
- Singularity analysis
- Visual servoing

---

## 6. Singularities

A singularity is a robot configuration where motion capability is lost.

Example:

```text
fully stretched arm
```

Near singularities:

- Joint velocities may become extremely large
- IK becomes unstable
- Cartesian control becomes problematic

---

### Symptoms

- Jerky motion
- Sudden joint spinning
- Planner failures
- Poor manipulability

---

### Difference Between Joint Limits and Singularities

| Joint Limits | Singularities |
|---|---|
| Physical restriction | Mathematical issue |
| Fixed range | Depends on posture |
| Defined manually | Emerges from geometry |

---

# 7. PlanningScene

## What Is PlanningScene?

`PlanningScene` is MoveIt's internal world model.

It contains:

- Robot state
- Collision objects
- Environment geometry
- Constraints
- Collision information

Think of it as:

> The robot's understanding of the world.

---

## What PlanningScene Is Used For

- Self collision checking
- World collision checking
- Constraint validation
- State validity checking
- Motion planning

---

## Self Collision Checking

Checks robot against itself.

Example:

```cpp
planning_scene.checkSelfCollision(...);
```

---

## World Collision Checking

Checks robot against environment.

Example:

```cpp
planning_scene.checkCollision(...);
```

---

## Allowed Collision Matrix (ACM)

Defines collisions that should be ignored.

Example:

```text
adjacent robot links
```

---

## State Validity

Checks:

- Collisions
- Constraints
- Feasibility

Example:

```cpp
planning_scene.isStateValid(...);
```

---

# 8. PlanningSceneMonitor

## What Is PlanningSceneMonitor?

`PlanningSceneMonitor` continuously updates the planning scene in real time.

It monitors:

- Joint states
- TF transforms
- Collision objects
- Octomap
- Sensor updates

Think of it as:

> A live continuously synchronized planning scene.

---

## Why It Is Recommended

A plain `PlanningScene` becomes outdated quickly.

`PlanningSceneMonitor` automatically keeps everything synchronized.

---

## Main Components

### State Monitor

Tracks:

```text
/joint_states
```

---

### Scene Monitor

Tracks planning scene updates.

---

### World Geometry Monitor

Tracks:

- Collision objects
- Octomap
- Sensors

---

## Thread Safety

MoveIt uses:

```cpp
LockedPlanningSceneRO
LockedPlanningSceneRW
```

for safe multi-threaded access.

---

# 9. Planning Scene ROS API

## Purpose

Allows external ROS nodes to modify MoveIt's planning scene.

Examples:

- Add obstacles
- Remove obstacles
- Attach objects
- Detach objects

---

## Scene Diff

Instead of sending the full scene, only changes are sent.

Example:

```text
add this box
```

instead of:

```text
entire world
```

---

## Collision Objects

Objects are added using:

```cpp
moveit_msgs::msg::CollisionObject
```

---

## AttachedCollisionObject

Used when robot grasps or carries objects.

Important because planner must account for:

- carried object geometry
- swept volume
- collision checking

---

## Touch Links

Defines robot links allowed to contact attached object.

Example:

```text
panda_hand
panda_leftfinger
panda_rightfinger
```

---

## Synchronous vs Asynchronous Updates

### Topic-based Updates

Asynchronous.

Fast but not guaranteed immediately.

---

### Service-based Updates

Synchronous.

Guarantees scene update completed before continuing.

---

# 10. Motion Planning API

## What Is Motion Planning?

Motion planning computes a collision-free trajectory from:

```text
start state -> goal state
```

---

## MotionPlanRequest

Describes the planning problem.

Contains:

- Start state
- Goal constraints
- Path constraints
- Workspace bounds
- Planning group

---

## MotionPlanResponse

Contains:

- Planned trajectory
- Error code
- Planning time

---

## Goal Constraints

Define the desired final state.

Example:

```text
move end-effector to target pose
```

---

## Path Constraints

Define rules that must remain valid throughout the motion.

Example:

```text
keep welding torch vertical
```

---

## Planning Flow

```text
Goal
  ↓
Planner
  ↓
Collision Checking
  ↓
Constraint Checking
  ↓
Trajectory
```

---

## Why Planning May Fail

- Collisions
- Unreachable target
- Singularities
- Impossible constraints
- Timeout

---

# 11. Motion Planning Pipeline

## What Is MotionPlanningPipeline?

A framework that manages the full planning workflow.

It handles:

- Request adapters
- Planner plugins
- Trajectory processing
- Time parameterization

---

## Pipeline Structure

```text
MotionPlanRequest
        ↓
Request Adapters
        ↓
Planner Plugin
        ↓
Trajectory Processing
        ↓
MotionPlanResponse
```

---

## Request Adapters

Adapters fix or improve requests/trajectories.

Examples:

| Adapter | Purpose |
|---|---|
| FixStartStateBounds | Clamp invalid joints |
| FixStartStateCollision | Resolve collisions |
| AddTimeParameterization | Add timestamps |

---

## Time Parameterization

Converts geometric paths into executable trajectories.

Adds:

- Timestamps
- Velocities
- Accelerations

---

## Why Pipelines Matter

Raw planner output alone is usually not executable.

Pipeline processing makes trajectories suitable for real robots.

---

# 12. move_group vs MotionPlanningPipeline

## move_group

High-level MoveIt runtime node.

Handles:

- Motion planning
- Trajectory execution
- Controller communication
- Planning scene monitoring
- ROS interfaces
- RViz interaction

Think of it as:

> The complete robot motion planning and execution system.

---

## MotionPlanningPipeline

Internal planning framework used by MoveIt.

Handles:

- Planner execution
- Request adapters
- Trajectory processing

Think of it as:

> The planning engine inside move_group.

---

## Relationship

```text
MoveGroupInterface
        ↓
move_group
        ↓
MotionPlanningPipeline
        ↓
Planner Plugin
```

---

## Key Difference

| move_group | MotionPlanningPipeline |
|---|---|
| Full runtime system | Planning subsystem |
| Executes trajectories | Generates trajectories |
| ROS interfaces | Internal framework |
| Controller communication | No controller interaction |
| RViz integration | No RViz integration |

---

## When To Use move_group

Use when:

- Controlling real robots
- Using RViz
- Building applications quickly
- Using MoveGroupInterface
- Performing trajectory execution

Best for most applications.

---

## When To Use MotionPlanningPipeline

Use when:

- Doing research
- Creating custom planning systems
- Benchmarking planners
- Writing custom planning workflows
- Understanding MoveIt internals

Mostly used in advanced/custom applications.

---

# 13. Overall MoveIt Architecture

```text
Sensors / ROS Topics
        ↓
PlanningSceneMonitor
        ↓
PlanningScene
        ↓
MotionPlanningPipeline
        ↓
Planner Plugin (OMPL/CHOMP/STOMP)
        ↓
Trajectory Processing
        ↓
move_group
        ↓
Controllers
        ↓
Robot Hardware
```

---

# 14. Important Practical Understanding

## RobotModel

Static robot structure.

---

## RobotState

Current robot configuration.

---

## PlanningScene

Current world representation.

---

## PlanningSceneMonitor

Keeps world representation updated.

---

## MotionPlanningPipeline

Generates valid robot trajectories.

---

## move_group

Complete robot motion planning and execution system.

