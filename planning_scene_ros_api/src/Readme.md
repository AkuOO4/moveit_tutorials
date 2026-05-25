# Planning Scene ROS API Tutorial Explanation

This tutorial demonstrates how to interact with the MoveIt planning scene using ROS 2 APIs in C++. The code walks through adding, attaching, detaching, and removing collision objects in the planning scene, using both topic and service interfaces. Below is a breakdown of the main steps and concepts:

## 1. Initialization
- Initializes the ROS 2 node and executor.
- Sets up the `rviz_visual_tools` for visualization and user prompts in RViz.

## 2. Planning Scene Publisher
- Creates a publisher for the `planning_scene` topic.
- Waits for at least one subscriber before proceeding (ensures RViz or MoveIt is ready).

## 3. Defining and Adding a Collision Object
- Defines an `AttachedCollisionObject` message representing a box to be attached to the robot's hand.
- Specifies the object's pose, dimensions, and the robot links that can touch it without triggering collisions.
- Publishes the object to the planning scene, adding it to the world at the hand's location.

## 4. Synchronous vs Asynchronous Updates
- Explains two ways to update the planning scene:
  - **Asynchronous:** Publishing diffs to the topic (used for most of the tutorial).
  - **Synchronous:** Using the `apply_planning_scene` service to ensure the update is applied before continuing.
- Demonstrates a synchronous service call to apply the planning scene diff and checks for success.

## 5. Attaching an Object to the Robot
- Removes the object from the world and attaches it to the robot's hand (simulating a pick operation).
- Updates the planning scene accordingly and prompts the user.

## 6. Detaching an Object from the Robot
- Detaches the object from the robot and reintroduces it into the world (simulating a place operation).
- Updates the planning scene and prompts the user.

## 7. Removing the Object from the World
- Removes the object from the planning scene entirely.
- Updates the planning scene and prompts the user.

## 8. Shutdown
- Shuts down the ROS node after the demo is complete.

---

## Key Concepts
- **Planning Scene:** Represents the robot, environment, and collision objects for motion planning.
- **Collision Objects:** Objects in the environment that the robot should avoid or interact with.
- **Attached Objects:** Objects held by the robot, which should be considered part of the robot for collision checking.
- **Diffs:** Only the changes (diffs) to the planning scene are sent, not the entire scene.
- **Visualization:** `rviz_visual_tools` is used for step-by-step visualization and user prompts in RViz.

## Usage
This tutorial is useful for understanding how to:
- Programmatically add, attach, detach, and remove objects in MoveIt's planning scene.
- Use both topic and service interfaces for planning scene updates.
- Integrate visualization and user interaction in RViz.

---

For more details, see the comments in the source code and the [MoveIt documentation](https://moveit.picknik.ai/).