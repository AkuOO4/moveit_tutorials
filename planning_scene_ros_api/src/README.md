# MoveIt Planning Scene ROS API

This document explains the concepts demonstrated in `planning_scene_ros_api_tutorial.cpp`. Where the `move_group_interface` tutorial uses the high-level `PlanningSceneInterface` helper, this tutorial communicates directly with the `move_group` node's planning scene using raw ROS topics and services — giving you full control over scene diffs.

---

## Table of Contents

1. [The Planning Scene and Diffs](#1-the-planning-scene-and-diffs)
2. [Async vs. Sync Scene Updates](#2-async-vs-sync-scene-updates)
3. [Adding a Collision Object to the World](#3-adding-a-collision-object-to-the-world)
4. [AttachedCollisionObject — The Unified Message](#4-attachedcollisionobject--the-unified-message)
5. [Attaching an Object to the Robot](#5-attaching-an-object-to-the-robot)
6. [Detaching an Object from the Robot](#6-detaching-an-object-from-the-robot)
7. [Removing an Object from the World](#7-removing-an-object-from-the-world)
8. [touch_links and the Grasp Collision Exception](#8-touch_links-and-the-grasp-collision-exception)
9. [Why Use the ROS API Directly?](#9-why-use-the-ros-api-directly)

---

## 1. The Planning Scene and Diffs

**What it is:** The `move_group` node maintains a *live* planning scene — the robot's current understanding of the world. You update it by publishing *diffs*: messages that describe only what has changed, not the full scene state.

**The key idea:** A diff is like a patch file. Instead of sending a complete world description every time you want to add a box, you send "add this one box." The `move_group` node merges the diff into its current scene.

```cpp
rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
    node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
```

**The `is_diff` flag:** Setting `planning_scene.is_diff = true` tells the `move_group` node to merge rather than replace. Without this flag, you would be asking the node to discard its entire scene and use your message as the new ground truth — almost never what you want.

```cpp
planning_scene.is_diff = true;
planning_scene_diff_publisher->publish(planning_scene);
```

**Two parts of a planning scene message:**
- `world` — collision objects and octomap data (the environment)
- `robot_state` — the robot's joint positions and attached collision objects

Both can be updated independently in the same diff.

---

## 2. Async vs. Sync Scene Updates

**What it is:** There are two ways to send a planning scene diff to `move_group`:

| Method | How | Behavior |
|---|---|---|
| **Topic publish** (async) | `planning_scene_diff_publisher->publish(scene)` | Returns immediately; diff applied later |
| **Service call** (sync) | `apply_planning_scene` service | Blocks until diff is confirmed applied |

**The key idea:** For interactive demos with human-paced `prompt()` pauses, async is fine — the diff will be applied before the human presses "next." For automated pipelines where you immediately plan after modifying the scene, you need sync to guarantee the planner sees your changes.

```cpp
// Sync approach
rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
    node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
planning_scene_diff_client->wait_for_service();
auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
request->scene = planning_scene;
auto response_future = planning_scene_diff_client->async_send_request(request).future.share();
```

**When async causes bugs:** A common mistake is publishing a collision object and immediately calling `plan()`. The planner uses the scene at the moment of planning — if the async update hasn't arrived yet, the object isn't there and the plan ignores it. The `ApplyPlanningScene` service eliminates this race.

---

## 3. Adding a Collision Object to the World

**What it is:** Placing a static obstacle in the planning scene so the planner will route around it.

```cpp
moveit_msgs::msg::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene.is_diff = true;
planning_scene_diff_publisher->publish(planning_scene);
```

**The key idea:** Objects live in `planning_scene.world.collision_objects`. Each object has:
- `id` — unique string identifier used for later operations (attach, detach, remove)
- `header.frame_id` — the coordinate frame the object's pose is defined in
- `primitives` — the geometric shapes (box, sphere, cylinder, cone)
- `primitive_poses` — the pose of each primitive relative to the frame
- `operation` — `ADD`, `REMOVE`, or `MOVE`

**Object identity:** The `id` field is how every subsequent operation (attach, remove) refers to this object. Choose meaningful, unique IDs.

---

## 4. AttachedCollisionObject — The Unified Message

**What it is:** `AttachedCollisionObject` is a single message that describes both an object's geometry (`object` field, same as `CollisionObject`) and its attachment to a robot link (`link_name` field).

```cpp
moveit_msgs::msg::AttachedCollisionObject attached_object;
attached_object.link_name = "panda_hand";
attached_object.object.header.frame_id = "panda_hand";
attached_object.object.id = "box";
```

**The key idea:** This one message type covers the full lifecycle — you can use it to add an object to the world (via `planning_scene.world`), attach it to the robot (via `planning_scene.robot_state`), and detach it, all with the same structure.

**`operation` field:** The `operation` field on the inner `object` drives what happens:
- `ADD` — create or update the object
- `REMOVE` — delete the object
- `MOVE` — update just the pose (not geometry)

---

## 5. Attaching an Object to the Robot

**What it is:** Simulating a grasp — the object stops being a world obstacle and starts being part of the robot's collision geometry.

Attaching requires two things to happen atomically:

1. **Remove** the object from the world (otherwise it exists in two places)
2. **Add** it to the robot state as an attached object

```cpp
// Step 1: mark the world object for removal
moveit_msgs::msg::CollisionObject remove_object;
remove_object.id = "box";
remove_object.operation = remove_object.REMOVE;

// Step 2: attach to robot state
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
planning_scene.robot_state.is_diff = true;
planning_scene_diff_publisher->publish(planning_scene);
```

**The key idea:** Both changes go in the same diff message. This guarantees `move_group` applies them together, so the scene is never in a state where the object is both a world obstacle and an attached object simultaneously.

**Effect on planning:** After attachment, the planner treats the box as part of the robot. Any joint configuration where the box collides with something in the world is now invalid — the robot effectively has a larger collision geometry.

---

## 6. Detaching an Object from the Robot

**What it is:** Simulating a release — the object goes back to being a world obstacle at its current pose.

Again, two things happen atomically:

1. **Detach** the object from the robot state
2. **Re-add** it to the world at the same position

```cpp
moveit_msgs::msg::AttachedCollisionObject detach_object;
detach_object.object.id = "box";
detach_object.link_name = "panda_hand";
detach_object.object.operation = attached_object.object.REMOVE;  // REMOVE from attachment

planning_scene.robot_state.attached_collision_objects.clear();
planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
planning_scene.world.collision_objects.push_back(attached_object.object);  // back to world
planning_scene.is_diff = true;
planning_scene_diff_publisher->publish(planning_scene);
```

**The key idea:** After detachment, the object reappears in the world at the position defined in `attached_object.object.primitive_poses`. In a real system, you would want this pose to reflect where the robot actually placed the object — meaning you'd compute the current world-frame pose of the object before detaching.

---

## 7. Removing an Object from the World

**What it is:** Permanently deleting an object from the planning scene so it no longer affects collision checking.

```cpp
planning_scene.world.collision_objects.clear();
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene_diff_publisher->publish(planning_scene);
```

**The key idea:** The `remove_object` message with `operation = REMOVE` is the deletion signal. You must clear the other fields of the diff (attached objects, etc.) before sending, or you'll accidentally undo other operations.

**Why clear before pushing:** If the `planning_scene` message still has a previously pushed `AttachedCollisionObject` from an earlier step and you publish again, `move_group` will re-apply that attachment as part of the same diff. The `clear()` calls are essential bookkeeping.

---

## 8. touch_links and the Grasp Collision Exception

**What it is:** When an object is attached to the gripper, the object is physically touching the gripper fingers. Without special handling, the collision checker would flag the grasp itself as a collision.

```cpp
attached_object.touch_links = std::vector<std::string>{
    "panda_hand", "panda_leftfinger", "panda_rightfinger"
};
```

**The key idea:** `touch_links` is a whitelist of robot links that are *allowed* to be in contact with the attached object. The collision checker skips collision tests between the object and any link in this list. Every other link — wrist, elbow, other arm links — still triggers a collision if the object hits them.

**Why you can't just disable all collisions:** If you disabled all collisions for the attached object, the robot could carry it into other obstacles freely. You want the object to be invisible to the gripper fingers (which are holding it) but still visible to everything else in the environment.

---

## 9. Why Use the ROS API Directly?

The `PlanningSceneInterface` in the `move_group_interface` tutorial provides convenience wrappers (`addCollisionObjects`, `attachObject`, etc.) that are easier to use. So why learn the raw API?

**Reasons to go lower-level:**
- **Atomic multi-object updates:** The raw diff lets you add, remove, and attach multiple objects in a single message, guaranteeing consistency. The high-level API makes separate calls.
- **Direct control over sync vs. async:** You choose whether to use the topic (async) or the service (sync) based on your timing requirements.
- **Non-MoveGroup nodes:** If you are writing a perception node or a world model manager that publishes scene updates — and it does not have a `MoveGroupInterface` — the topic API is your only option.
- **Debugging:** Understanding what messages `move_group` actually receives helps diagnose scene update issues.

---

## Quick Reference

| Operation | Mechanism | Key Fields |
|---|---|---|
| Add world object | `world.collision_objects` + `operation=ADD` | `id`, `primitives`, `primitive_poses` |
| Remove world object | `world.collision_objects` + `operation=REMOVE` | `id` |
| Attach to robot | `world` REMOVE + `robot_state.attached_collision_objects` ADD | `link_name`, `touch_links` |
| Detach from robot | `robot_state` REMOVE + `world` ADD | `link_name`, `object.id` |
| Async update | Publish to `planning_scene` topic | `is_diff = true` |
| Sync update | Call `apply_planning_scene` service | Wait for response |
