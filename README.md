# 🤖 Intelligent Robotics — Assignment 1  
## Group 07  
- Giacomo Vianello (ID: 2140028)  
- Salvatore Ferracane (ID: 2154255)  

## 📘 Project Overview  

This repository contains our solution to [Assignment 1](Assignment_1.pdf) for the *Intelligent Robotics* course at the University of Padua. Our implementation builds upon the base repository [`ir_2526`](https://github.com/PieroSimonet/ir_2526.git), which is included under the `src/` directory.

The project is structured as a ROS 2 workspace and includes all necessary components to run, test, and extend our assignment solution.

## 🏃‍♂️‍➡️ Run the Project
The first step is to build and source the entire ROS 2 workspace:
```
colcon build
source install/setup.bash
```
To run our project, we provide a structured and configurable launch file called `global.launch.py`. It can be started with:
```
ros2 launch assignment_1_07 global.launch.py
```
This launch file orchestrates the whole assignment setup. Specifically, it:
- Includes the base launch file from the provided repository (`ir_launch/assignment_1.launch.py`).
- Starts the **AprilTag detection node** with the correct topic remappings and parameters.
- Runs the **Nav2Orchestrator node**, which automatically initializes the localization and navigation stacks and publishes the initial pose to AMCL.
- Launches our **Goal Selector node**, which computes navigation goals from detected tags and republishes them every 5 seconds.
- Launches the **Goal Sender node**, which listens to `/goal_pose_raw` and sends the goal to Nav2 via the `navigate_to_pose` action.
- **TO BE COMPLETED**

This way, a single command brings up the entire system, ready for testing and demonstration.

## 📷 Apriltags and Camera Connections

We rely on the external package `apriltag_ros`, which uses the AprilTag library to detect tags in camera images and publish their pose, ID, and metadata.
Normally, enabling AprilTag detection requires adjusting the default configuration file (`tags_36h11.yaml`) inside the `apriltag_ros` package. For example, one could run:
```sh
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/rgb_camera/image \
  -r camera_info:=/rgb_camera/camera_info \
  --params-file $(ros2 pkg prefix apriltag_ros)/share/apriltag_ros/cfg/tags_36h11.yaml
```
However, instead of modifying the default configuration file, we created our own custom launch file (`global.launch.py`) inside the `assignment_1_07` package. This approach has several advantages:
- **Correct parameters**: The launch file automatically loads the configuration from `assignment_1_07/config/apriltag_params.yaml`, which specifies the actual tag size (**0.05 m × 0.05 m**) and other relevant parameters.
- **Maintainability**: If the AprilTag setup changes in the future (e.g., different tag family or size), we only need to update our YAML file without touching the external package.
- **Integration**: The launch file ensures that the AprilTag node runs alongside the rest of our system (goal selector, orchestrator, etc.), so everything is initialized consistently.

### Published topics
Once running, the node provides:
- `/tf` - publishes transforms (`tf2_msgs/msg/TFMessage`) for each detected tag.
- `detections` - publishes detection results (`apriltag_msgs/msg/AprilTagDetectionArray`) including tag IDs, poses, and metadata

### Visualization Tips
To confirm that the node is working:
1. Launch `RViz2`
2. Select `odom` as fixed frame.
3. Add a **Display -> Image** and select `/rgb_camera/image` to see the camera feed.
4. Add a **Display -> TF** to visualize the coordinate frames of detected tags (e.g., `tag36h11:0`).

Now we should see the two apriltag frames (`tag36h11:1`, and `tag36h11:10`) wrt the `odom` frame. 

Notice that the tables (cylindrical objects) are not in field of view of the Camera. Hence, we will need to localize them using the turlebot sensors.


## 🎼 Nav2 Orchestrator Node
The Nav2Orchestrator node automates the initialization of the full navigation stack by interacting directly with the lifecycle managers of both localization and navigation. Instead of requiring manual service calls or user input in RViz, this node ensures that all components are correctly configured and activated in sequence.

### Behavior and design

Our design is motivated by a key observation: after activating the localization stack via `/lifecycle_manager_localization/manage_nodes`, the navigation lifecycle manager (server) will reject a `STARTUP` request (`success=False`) if the system has not yet received a valid initial pose estimate. We have no clue on why this happens, but, to address this, we structured the orchestrator in 4 execution steps:
1. Start localization lifecycle manager.
2. Publish the initial pose to AMCL.
3. Wait briefly for AMCL to process the initial pose.
4. Start navigation lifecycle manager.

More in detail, we addressed:
- **Lifecycle management**: 
  - Connects to `/lifecycle_manager_localization/manage_nodes` and `/lifecycle_manager_navigation/manage_nodes`.
  - Sends `STARTUP` commands to bring all managed nodes into the `active` state.

- **Initial pose publication**:
  - Publishes a `geometry_msgs/PoseWithCovarianceStamped` message on `/initialpose` immediately after localization is started.
  - This initializes AMCL without requiring the user to manually set the “2D Pose Estimate” in RViz.
  - The pose (x, y, yaw) can be customized via ROS parameters.

Additionally, the node includes configurable timeouts for service discovery and service calls, along with detailed logging for each lifecycle transition and initial pose publication. This design guarantees that the navigation stack is only activated once localization is stable, resulting in a more reliable and fully automated startup procedure.

### Parameters

- **initial_x** (double, default: 0.0): X coordinate of the initial pose in the map frame.
- **initial_y** (double, default: 0.0): Y coordinate of the initial pose in the map frame.
- **initial_yaw** (double, default: 0.0): Orientation (yaw, in radians) of the initial pose.
- **service_wait_timeout_sec** (double, default: 10.0): Maximum time to wait for lifecycle services to become available.
- **call_timeout_sec** (double, default: 10.0): Maximum time to wait for a lifecycle service call to complete.

These parameters can be adjusted in the provided launch file (`global.launch.py`).


## 🎯 Goal Selector Node
The Goal Selector node reads AprilTag detections and publishes a navigation goal computed from the detected tags. It is designed to work with the `apriltag_node` that publishes tag frames on `/tf` (preferred) and the `/detections` topic.

### Behavior and design
- **Input**: subscribes to `/detections` (`apriltag_msgs/AprilTagDetectionArray`) to learn which tag IDs are visible.

- **Pose acquisition**: obtains each tag pose by doing a TF lookup from the configured target_frame (`odom` or `map`) to the tag frame (e.g. `tag36h11:1`).

- **Goal calculation**: computes a goal as the midpoint between two detected tags (first two detections). The published goal is a `geometry_msgs/PoseStamped` in the chosen target frame.

- **Output**: republishes the computed goal every 5 seconds on `/goal_pose_raw` (`geometry_msgs/PoseStamped`). 

### Parameters
- **target_frame** (string, default: `"odom"`): frame in which the goal is published and in which TF lookups are performed.
- **tag_frame_prefix** (string, default: `"tag36h11:"`): prefix used to build the tag frame name from the integer tag ID (prefix + ID, e.g. `tag36h11:1`). Must match the names in `apriltag_node` configuration.
- **tf_timeout_sec** (double, default: `0.3`): timeout used when waiting for the TF lookup.

These parameters can be adjusted in the provided launch file (`global.launch.py`).

## 📡 Goal Sender Node

The Goal Sender node listens to `/goal_pose_raw` and acts as an action client for Nav2. Whenever a new goal is received, it sends it to the `navigate_to_pose` action server, triggering navigation.

### Behavior and design
- **Input**: subscribes to `/goal_pose_raw` (`geometry_msgs/PoseStamped`).
- **Action client**: connects to `navigate_to_pose` (Nav2).
- **Execution**: sends the goal once, waits for Nav2 to complete, and logs the result (success, aborted, canceled).

This separation of responsibilities ensures that the Goal Selector only computes goals, while the Goal Sender handles navigation requests.