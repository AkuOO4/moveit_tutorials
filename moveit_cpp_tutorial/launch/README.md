# launch/moveit_cpp_tutorial.launch.py

Launches the complete MoveItCpp demo environment: the demo node, RViz, TF publishers, ros2_control, and all required controllers.

## Nodes started

### `moveit_cpp_tutorials`
- **Package:** `moveit_cpp_tutorial`
- **Executable:** `moveit_cpp_tutorials`
- **Parameters:** Full MoveIt config dict (robot description, SRDF, kinematics, OMPL, MoveItCpp settings) injected via `moveit_config.to_dict()`

### `rviz2`
- Loads `rviz/moveit_cpp_tutorial.rviz` for a pre-configured display
- Receives `robot_description` and `robot_description_semantic` so the robot model renders correctly
- Output sent to log (not screen) to keep the terminal clean

### `static_transform_publisher`
- Publishes a zero transform from `world` → `panda_link0`
- Required because the Panda URDF root is `panda_link0` but MoveIt expects a `world` frame as the planning frame

### `robot_state_publisher`
- Reads `robot_description` (URDF) and broadcasts joint-based TF transforms
- Needed by RViz and MoveIt to track link poses

### `ros2_control_node`
- Loads `panda_moveit_config/config/ros2_controllers.yaml`
- Uses the **FakeSystem** hardware interface — simulates joint state feedback without real hardware
- Remaps `/controller_manager/robot_description` → `/robot_description` so it picks up the URDF from the standard topic

### Controller spawners
Three controllers are loaded via `ExecuteProcess` + `ros2 run controller_manager spawner`:

| Controller | Purpose |
|------------|---------|
| `panda_arm_controller` | Joint trajectory controller for the 7-DOF arm |
| `panda_hand_controller` | Joint trajectory controller for the gripper fingers |
| `joint_state_broadcaster` | Re-publishes hardware joint states on `/joint_states` |

## MoveIt configuration

The config is built with `MoveItConfigsBuilder` targeting the `panda` robot from `panda_moveit_config`:

| Builder call | File loaded |
|--------------|-------------|
| `robot_description` | `config/panda.urdf.xacro` |
| `trajectory_execution` | `config/gripper_moveit_controllers.yaml` |
| `planning_pipelines` | OMPL only |
| `moveit_cpp` | `config/moveit_cpp.yaml` |

`moveit_cpp.yaml` configures the planning scene monitor, async planning, and other MoveItCpp-specific settings.

## Launch order

The `LaunchDescription` adds nodes in this order:

```
static_tf → robot_state_publisher → rviz_node → moveit_cpp_node → ros2_control_node
  + [panda_arm_controller spawner, panda_hand_controller spawner, joint_state_broadcaster spawner]
```

The demo node sleeps for 1 second at startup (see source) to wait for joint states to be published before initializing MoveItCpp.
