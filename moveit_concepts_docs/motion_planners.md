# MoveIt Motion Planners and OMPL Planners

# 1. Introduction

MoveIt is a motion planning framework for robotic manipulation. It provides:

* Robot modeling
* Collision checking
* Kinematics
* Constraints handling
* Trajectory execution
* Planning pipelines

MoveIt itself is not a planner. It integrates different planning libraries through plugin-based planning pipelines.

Major planning frameworks used in MoveIt:

* OMPL (Open Motion Planning Library)
* CHOMP
* STOMP
* Pilz Industrial Motion Planner
* TrajOpt
* Cartesian path interpolation utilities

---

# 2. Planner Categories in MoveIt

| Category                 | Examples               | Main Idea                                 |
| ------------------------ | ---------------------- | ----------------------------------------- |
| Sampling-based           | OMPL planners          | Random exploration in configuration space |
| Optimization-based       | CHOMP, STOMP, TrajOpt  | Improve trajectory iteratively            |
| Industrial deterministic | Pilz                   | Predefined industrial-style motions       |
| Cartesian interpolation  | computeCartesianPath() | Direct end-effector interpolation         |

---

# 3. Major Planning Frameworks in MoveIt

# 3.1 OMPL (Open Motion Planning Library)

OMPL is the default and most widely used planning framework in MoveIt.

## Characteristics

* Sampling-based
* Collision-aware
* Fast planning
* Good for high-dimensional robots
* Non-deterministic
* Usually requires path smoothing

## Typical Use Cases

* General robotic manipulation
* Industrial robot arms
* Mobile manipulators
* Obstacle avoidance

## Most Common Planner

* RRTConnect

---

# 3.2 CHOMP

CHOMP (Covariant Hamiltonian Optimization for Motion Planning) is an optimization-based planner.

## Characteristics

* Smooth trajectories
* Gradient-based optimization
* Sensitive to local minima
* Slower than OMPL

## Best Use Cases

* Smooth industrial motion
* Constrained environments
* Trajectory refinement

---

# 3.3 STOMP

STOMP (Stochastic Trajectory Optimization for Motion Planning) uses stochastic optimization instead of gradients.

## Characteristics

* Smooth trajectories
* Robust against noisy cost functions
* Handles difficult optimization better than CHOMP in some cases

## Best Use Cases

* Industrial motion planning
* Smooth trajectory generation
* Constrained optimization

---

# 3.4 Pilz Industrial Motion Planner

Designed for deterministic industrial robot motion.

## Motion Types

* PTP (Point-to-Point)
* LIN (Linear)
* CIRC (Circular)

## Best Use Cases

* Welding
* Pick-and-place
* Straight-line tool motion

## Limitations

* Does not actively search around obstacles
* Collision checked after trajectory generation

---

# 3.5 Cartesian Path Planning

MoveIt provides Cartesian interpolation through:

```cpp
computeCartesianPath()
```

## Characteristics

* Exact Cartesian tool motion
* Deterministic
* No global search

## Best Use Cases

* Welding
* Painting
* Surface following

---

# 4. OMPL Planner Types

OMPL planners are mainly divided into:

| Planner Family       | Examples        |
| -------------------- | --------------- |
| Tree-based           | RRT, RRTConnect |
| Roadmap-based        | PRM             |
| Optimal planners     | RRT*, PRM*, FMT |
| Lazy planners        | LazyPRM         |
| Exploration planners | KPIECE, EST     |
| Cost-aware planners  | TRRT            |

---

# 5. OMPL Planners

# 5.1 RRT (Rapidly Exploring Random Tree)

## Core Idea

Builds a tree by randomly exploring the configuration space.

## Advantages

* Simple
* Fast in high-dimensional spaces
* Good exploration capability

## Weaknesses

* Poor path quality
* Non-optimal
* Jerky trajectories

## Best Use Cases

* Basic motion planning
* Feasibility testing
* Research prototypes

---

# 5.2 RRTConnect

## Core Idea

Uses two trees:

* One from the start
* One from the goal

Attempts to connect them rapidly.

## Advantages

* Extremely fast
* Very reliable
* Excellent for robot arms

## Weaknesses

* Non-optimal
* Path quality can be poor

## Best Use Cases

* General MoveIt planning
* Industrial manipulators
* Obstacle avoidance

## Industry Usage

Most commonly used MoveIt planner.

---

# 5.3 RRT*

## Core Idea

RRT with continuous tree rewiring to improve path quality.

## Advantages

* Asymptotically optimal
* Better path quality

## Weaknesses

* Slower
* Computationally expensive

## Best Use Cases

* Offline planning
* Optimal path generation
* Research

---

# 5.4 PRM (Probabilistic Roadmap)

## Core Idea

Builds a reusable roadmap graph of valid robot states.

## Advantages

* Excellent for repeated queries
* Efficient in static environments

## Weaknesses

* Slow roadmap creation
* Memory intensive

## Best Use Cases

* Factory cells
* Repeated planning tasks
* Static environments

---

# 5.5 PRM*

Optimal version of PRM.

## Advantages

* Higher-quality paths
* Asymptotically optimal

## Weaknesses

* More computationally expensive

## Best Use Cases

* Offline optimal planning
* Long-term industrial workcells

---

# 5.6 LazyPRM / LazyPRM*

## Core Idea

Delays collision checking until required.

## Advantages

* Faster roadmap generation
* Reduced collision-checking cost

## Weaknesses

* Invalid paths may be discovered late

## Best Use Cases

* Complex collision environments
* Large planning scenes

---

# 5.7 KPIECE

(Kinematic Planning by Interior-Exterior Cell Exploration)

## Core Idea

Projects high-dimensional states into lower-dimensional exploration cells.

## Advantages

* Good narrow-passage handling
* Effective in constrained spaces

## Weaknesses

* Harder to tune
* Projection selection important

## Best Use Cases

* Humanoids
* Cluttered environments
* Constrained manipulators

---

# 5.8 BKPIECE / LBKPIECE

Variants of KPIECE.

## BKPIECE

Bidirectional KPIECE.

### Advantage

* Faster convergence

## LBKPIECE

Lazy bidirectional KPIECE.

### Advantage

* Reduced collision checking

---

# 5.9 EST (Expansive Space Trees)

## Core Idea

Expands preferentially into sparsely explored regions.

## Advantages

* Good exploration capability

## Weaknesses

* Slower than modern planners
* Poor path quality

## Best Use Cases

* Exploration-heavy problems
* Research applications

---

# 5.10 SBL (Single-query Bidirectional Lazy)

## Core Idea

Bidirectional planner with lazy collision checking.

## Advantages

* Reduced collision checking

## Weaknesses

* Lower robustness than RRTConnect

## Best Use Cases

* Expensive collision-checking environments

---

# 5.11 TRRT (Transition-based RRT)

## Core Idea

RRT combined with cost-map optimization.

## Advantages

* Cost-aware planning
* Energy-aware trajectories

## Weaknesses

* Parameter sensitive
* Slower

## Best Use Cases

* Terrain navigation
* Cost-aware motion planning

---

# 5.12 FMT / BFMT

(Fast Marching Trees)

## Core Idea

Batch sampling with dynamic-programming-like expansion.

## Advantages

* High-quality paths
* Good asymptotic performance

## Weaknesses

* Higher memory usage
* Less suited for dynamic environments

## Best Use Cases

* Offline optimization
* Large planning spaces

---

# 5.13 SST

Sparse Stable Trees.

Mostly used in kinodynamic planning.

## Advantages

* Handles system dynamics
* Near-optimal trajectories

## Weaknesses

* Complex tuning
* Slower planning

## Best Use Cases

* Drones
* Dynamic systems
* Mobile robots

---

# 6. Planner Comparison Summary

| Planner    | Speed     | Path Quality | Optimal    | Repeated Queries | Narrow Passages |
| ---------- | --------- | ------------ | ---------- | ---------------- | --------------- |
| RRT        | Fast      | Poor         | No         | No               | Moderate        |
| RRTConnect | Very Fast | Moderate     | No         | No               | Good            |
| RRT*       | Slow      | Good         | Yes        | No               | Good            |
| PRM        | Moderate  | Moderate     | No         | Yes              | Moderate        |
| PRM*       | Slow      | Good         | Yes        | Yes              | Moderate        |
| LazyPRM    | Fast      | Moderate     | Optional   | Yes              | Moderate        |
| KPIECE     | Moderate  | Moderate     | No         | No               | Good            |
| EST        | Moderate  | Poor         | No         | No               | Good            |
| TRRT       | Slow      | Good         | Cost-aware | No               | Moderate        |

---

# 7. Recommended Planner Selection

| Application                  | Recommended Planner |
| ---------------------------- | ------------------- |
| General robot arm planning   | RRTConnect          |
| Fast feasible planning       | RRTConnect          |
| Optimal path generation      | RRT*                |
| Repeated factory planning    | PRM                 |
| Expensive collision checking | LazyPRM             |
| Narrow passage planning      | KPIECE              |
| Cost-aware motion            | TRRT                |
| Dynamic systems              | SST                 |
| Offline optimization         | FMT / PRM*          |

---

# 8. Industrial Robotics Recommendations

## Welding Robots

Recommended:

* Pilz LIN
* Cartesian planning
* OMPL + STOMP/CHOMP smoothing

## Humanoid Welding

Recommended:

* KPIECE
* RRTConnect

## Fixed Factory Cell

Recommended:

* PRM
* PRM*

## Online Replanning

Recommended:

* RRTConnect

---

# 9. Typical Industrial Planning Pipeline

Most real systems combine multiple planners and processing stages:

1. OMPL planner generates feasible path
2. CHOMP/STOMP smooths trajectory
3. Time parameterization generates executable motion
4. Servo control corrects execution errors

---

# 10. MoveIt Planner Configuration

Example OMPL configuration:

```yaml
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect

  PRMkConfigDefault:
    type: geometric::PRM

  RRTstarkConfigDefault:
    type: geometric::RRTstar
```

Selecting planner in code:

```cpp
move_group.setPlannerId("RRTConnectkConfigDefault");
```

Selecting planning pipeline:

```cpp
move_group.setPlanningPipelineId("ompl");
```

---

# 11. Key Takeaways

* RRTConnect is the most commonly used MoveIt planner.
* PRM is best for repeated planning in static environments.
* RRT* and PRM* provide optimal paths but are slower.
* KPIECE is useful for narrow passages and complex articulated systems.
* Pilz planners are preferred for deterministic industrial motions.
* CHOMP/STOMP are often used for trajectory smoothing and optimization.
* Real industrial systems usually combine multiple planning approaches.
