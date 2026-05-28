# Inverse Kinematics (IK) Solvers in MoveIt

# 1. What is Inverse Kinematics (IK)

Inverse Kinematics (IK) is the process of computing robot joint values required to achieve a desired end-effector pose.

The desired pose usually contains:

* Position:

  * x
  * y
  * z

* Orientation:

  * roll
  * pitch
  * yaw

The IK solver computes:

```math
f(q) = T
```

Where:

* `q` → joint configuration
* `f(q)` → forward kinematics function
* `T` → target end-effector pose

---

# 2. Forward Kinematics vs Inverse Kinematics

| Type                    | Input        | Output            |
| ----------------------- | ------------ | ----------------- |
| Forward Kinematics (FK) | Joint values | End-effector pose |
| Inverse Kinematics (IK) | Desired pose | Joint values      |

---

# 3. Why IK is Important in MoveIt

MoveIt uses IK heavily in:

* Motion planning
* Cartesian path planning
* Grasp planning
* Servoing
* Manipulation
* Trajectory generation
* Constraint planning

A motion planner may call the IK solver:

* hundreds
* thousands
* or millions of times

during planning.

Thus IK performance strongly affects:

* planning speed
* planner success rate
* trajectory quality
* realtime capability

---

# 4. Types of IK Solvers

IK solvers are broadly divided into two categories:

## 4.1 Analytical IK

Solves equations symbolically using closed-form mathematics.

### Characteristics

* Extremely fast
* Deterministic
* Exact solutions
* Constant solve time

### Drawbacks

* Difficult to generate
* Limited robot compatibility
* Hard to maintain
* Poor flexibility

### Example

* IKFast

---

## 4.2 Numerical IK

Uses iterative optimization techniques.

### Characteristics

* Flexible
* Works for almost any robot
* Easier setup

### Drawbacks

* Slower
* Can fail to converge
* Sensitive to initial seed
* May get stuck in local minima

### Examples

* KDL
* LMA
* TracIK
* PickIK

---

# 5. Redundancy in IK

Redundancy means:

> The robot has more degrees of freedom (DOF) than required for the task.

A full 3D pose requires:

```math
6 \text{ DOF}
```

If the robot has:

* 6 DOF → non-redundant
* 7+ DOF → redundant

---

## Example

A 7DOF robot arm can achieve the same end-effector pose using multiple elbow configurations.

For one hand pose:

* elbow up
* elbow down
* folded posture
* stretched posture

may all work.

Thus:

```math
One pose → many joint solutions
```

---

## Why Redundancy is Useful

Extra DOF can help with:

* obstacle avoidance
* singularity avoidance
* smoother motion
* joint limit avoidance
* collision avoidance
* posture optimization

---

# 6. IK Solvers Available in MoveIt

| Solver    | Type                   | Speed          | Reliability     | Best Use            |
| --------- | ---------------------- | -------------- | --------------- | ------------------- |
| KDL       | Numerical              | Medium         | Medium          | General use         |
| LMA       | Numerical Optimization | Slow-Medium    | High            | Precision tasks     |
| TracIK    | Hybrid Numerical       | Fast           | Very High       | Production robotics |
| IKFast    | Analytical             | Extremely Fast | High            | Industrial robots   |
| PickIK    | Optimization-Based     | Medium         | Excellent       | Redundant robots    |
| Cached IK | Cache Wrapper          | Very Fast      | Depends backend | Repeated queries    |

---

# 7. KDL IK Solver

## Plugin

```yaml
kdl_kinematics_plugin/KDLKinematicsPlugin
```

## Type

Jacobian-based numerical IK.

Uses iterative Jacobian pseudo-inverse methods.

Core equation:

```math
\Delta q = J^\dagger \Delta x
```

Where:

* `J` → Jacobian
* `J†` → pseudo inverse

---

## Advantages

* Default MoveIt solver
* Easy setup
* Works for most robots
* Good for learning
* No preprocessing needed

---

## Drawbacks

* Slower than analytical IK
* Can fail near singularities
* Sensitive to seed state
* Weak redundancy handling

---

## Best Use Cases

* Research
* Prototyping
* Learning MoveIt
* Simple manipulators

---

# 8. LMA Solver (Levenberg-Marquardt)

## Plugin

```yaml
lma_kinematics_plugin/LMAKinematicsPlugin
```

---

## Type

Damped least-squares optimization.

Uses:

* Gradient descent
* Gauss-Newton optimization

Equation:

```math
\Delta q = (J^T J + \lambda I)^{-1} J^T e
```

---

## Advantages

* Better near singularities
* More stable
* Better precision
* Good orientation convergence

---

## Drawbacks

* Slower
* Still iterative
* Not ideal for realtime systems

---

## Best Use Cases

* Welding
* Assembly
* Precision insertion
* Constrained Cartesian motion

---

# 9. TracIK

## Plugin

```yaml
trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

---

## Type

Hybrid numerical IK.

Runs two solvers in parallel:

1. Newton-based solver
2. SQP optimization solver

Returns whichever converges first.

---

## Advantages

* Better success rate than KDL
* Faster convergence
* Better joint-limit handling
* More robust near singularities
* Good realtime performance

---

## Drawbacks

* Still iterative
* Slightly more setup complexity
* Solve time not perfectly deterministic

---

## Best Use Cases

* Production robotics
* Mobile manipulators
* Realtime planning
* Dynamic environments

---

## Practical Note

TracIK is often considered a better default than KDL for real-world robotics.

---

# 10. IKFast

## Type

Analytical closed-form IK.

Generated offline using OpenRAVE.

---

## Advantages

* Extremely fast
* Microsecond solve times
* Deterministic
* Exact solutions
* Excellent for repeated queries

---

## Drawbacks

* Difficult setup
* Requires code generation
* Limited flexibility
* Robot changes require regeneration
* Limited support for highly redundant robots

---

## Best Use Cases

* Industrial robot arms
* Pick-and-place
* Bin picking
* High-frequency planning
* Factory automation

---

## Common Industrial Usage

Frequently used for:

* UR robots
* ABB robots
* KUKA robots
* Fanuc robots

---

# 11. PickIK

## Plugin

```yaml
pick_ik/PickIkPlugin
```

---

## Type

Optimization-based global IK solver.

Uses:

* Gradient descent
* Evolutionary optimization
* Cost functions

---

## Advantages

* Excellent redundancy handling
* Supports optimization objectives
* Good for constrained planning
* Better whole-body motion
* Highly configurable

---

## Can Optimize For

* joint centering
* smoothness
* manipulability
* collision avoidance
* minimal joint movement

---

## Drawbacks

* More computationally expensive
* More tuning required
* More complex setup

---

## Best Use Cases

* 7DOF manipulators
* Humanoids
* Mobile manipulators
* Whole-body planning
* Complex constrained tasks

---

# 12. Cached IK

## Type

Caching wrapper around another IK solver.

Stores previously computed solutions.

---

## Advantages

* Very fast repeated queries
* Improves planning performance
* Useful in structured environments

---

## Drawbacks

* Not standalone
* Depends on backend solver
* Less useful in highly dynamic scenes

---

# 13. Comparison Summary

| Solver    | Type                   | Speed          | Strength                | Weakness                  |
| --------- | ---------------------- | -------------- | ----------------------- | ------------------------- |
| KDL       | Numerical              | Medium         | Easy setup              | Weak near singularities   |
| LMA       | Numerical Optimization | Slow           | High precision          | Computationally expensive |
| TracIK    | Hybrid Numerical       | Fast           | Reliability             | Non-deterministic timing  |
| IKFast    | Analytical             | Extremely Fast | Realtime industrial use | Hard setup                |
| PickIK    | Optimization           | Medium         | Redundancy optimization | More tuning               |
| Cached IK | Cache Wrapper          | Very Fast      | Repeated queries        | Backend dependent         |

---

# 14. Choosing an IK Solver

| Requirement             | Recommended Solver |
| ----------------------- | ------------------ |
| Beginner learning       | KDL                |
| General production use  | TracIK             |
| Maximum speed           | IKFast             |
| Precision tasks         | LMA                |
| Redundant robots        | PickIK             |
| Humanoids               | PickIK             |
| Industrial manipulators | IKFast             |
| Mobile manipulators     | TracIK             |

---

# 15. Typical Industry Choices

| Scenario            | Common Choice         |
| ------------------- | --------------------- |
| UR robots           | IKFast / TracIK       |
| Franka Panda        | TracIK / PickIK       |
| Humanoids           | PickIK                |
| Industrial welding  | IKFast + optimization |
| Research labs       | KDL                   |
| Mobile manipulators | TracIK                |

---

# 16. MoveIt IK Architecture

```text
Motion Planner
      ↓
Planning Request
      ↓
MoveIt Kinematics Plugin Interface
      ↓
IK Solver Plugin
(KDL / TracIK / IKFast / PickIK)
      ↓
Joint Solution
```

---

# 17. Important Practical Insight

Motion planning performance is heavily dependent on IK performance.

Fast IK:

* faster planning
* better realtime capability
* more planning attempts
* improved grasp sampling

This is why industrial systems often prefer:

* IKFast
* TracIK
* PickIK

instead of the default KDL solver.

---

# 18. Recommended Learning Order

Recommended progression:

1. Forward Kinematics
2. Jacobians
3. KDL
4. TracIK
5. IKFast
6. Redundancy resolution
7. Optimization-based IK
8. Whole-body IK

---

# 19. Useful References

## MoveIt Documentation

* https://moveit.picknik.ai/

## IKFast

* https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html

## TracIK

* https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html

## PickIK

* https://github.com/PickNikRobotics/pick_ik

## OpenRAVE IKFast

* http://openrave.org/docs/latest_stable/openravepy/ikfast/

---

# 20. Short Summary

| Solver    | Main Idea                         |
| --------- | --------------------------------- |
| KDL       | Basic numerical IK                |
| LMA       | Stable damped optimization        |
| TracIK    | Robust hybrid IK                  |
| IKFast    | Analytical high-speed IK          |
| PickIK    | Optimization-based intelligent IK |
| Cached IK | Cached acceleration layer         |

---
