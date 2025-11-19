# 🤖 Intelligent Robotics — Assignment 1  
## Group 07  
- Giacomo Vianello (ID: 2140028)  
- Salvatore Ferracane (ID: 2154255)  

## 📘 Project Overview  

This repository contains our solution to [Assignment 1](Assignment_1.pdf) for the *Intelligent Robotics* course at the University of Padua. Our implementation builds upon the base repository [`ir_2526`](https://github.com/PieroSimonet/ir_2526.git), which is included under the `src/` directory.

The project is structured as a ROS 2 workspace and includes all necessary components to run, test, and extend our assignment solution.

## 📌 Workspace

This project uses a **hybrid ROS2 workspace** to develop nodes in both **C++** and **Python**, ensuring flexibility and avoiding compatibility issues.  
The package structure, together with the `CMakeLists.txt` and `package.xml`, has been adapted following the guidelines from this [tutorial](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/).

📂 Package Structure

```
/IR_assignment_1_07/src/assignment_1_07
├── assignment_1_07
│   ├── __init__.py
│   └── table_detection_node.py
├── CMakeLists.txt
├── config
│   └── apriltag_params.yaml
├── launch
│   └── global.launch.py
├── LICENSE
├── package.xml
└── src
    ├── goal_selector.cpp
    └── nav2_orchestrator.cpp
```

where:
- All **Python nodes** are placed inside the `/assignment_1_07` folder (executables).  
- All **C++ nodes** are placed inside the `/src` folder.  

**NOTA:** 
Python executables must include the shebang:
```
#!/usr/bin/env python3
```



## 🏃‍♂️‍➡️ Run the Project
The first step is to build and source the entire ROS 2 workspace:

colcon build
source install/setup.bash

To run our project, we provide a structured and configurable launch file called `global.launch.py`. It can be started with:

ros2 launch assignment_1_07 global.launch.py

This launch file orchestrates the whole assignment setup. Specifically, it:
- Includes the base launch file from the provided repository (`ir_launch/assignment_1.launch.py`).
- Starts the **AprilTag detection node** with the correct topic remappings and parameters.
- Runs the **Nav2Orchestrator node**, which automatically initializes the localization and navigation stacks and publishes the initial pose to AMCL, and signals readiness.
- Launches the **Goal Selector node**, which works using both TF lookups of detected tags and a **custom service** to deliver the computed goal to other  nodes.
- Launches the **Goal Sender node**, which calls the `/get_current_goal` service, retrieves the latest computed goal, and sends it through the Nav2 action `navigate_to_pose`.
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

Notice that the tables (cylindrical objects) are not in field of view of the Camera. Hence, we will need to localize them using the turltebot sensors.


## 🎼 Nav2 Orchestrator Node
The **Nav2Orchestrator** node automates the initialization of the full navigation stack by interacting directly with the lifecycle managers of both localization and navigation. Instead of requiring manual service calls or user input in RViz, this node ensures that all components are correctly configured and activated in sequence, and publishes a readiness signal for the rest of the system.

### Behavior and design

Our design is motivated by a key observation: after activating the localization stack via `/lifecycle_manager_localization/manage_nodes`, the navigation lifecycle manager will reject a `STARTUP` request (`success=false`) if the system has not yet received a valid initial pose estimate. To address this, the orchestrator executes the following four steps:

1. Start the localization lifecycle manager.  
2. Publish the initial pose to AMCL.  
3. Wait briefly for AMCL to process the initial pose.  
4. Start the navigation lifecycle manager.  

Once both lifecycle managers report success, the orchestrator publishes a `std_msgs/Bool` message on the topic `/nav2_ready`. This signal is used by the **GoalSelector** and **GoalSender** nodes to activate their logic only when Nav2 is fully operational.

### Features
- **Lifecycle management**: connects to `/lifecycle_manager_localization/manage_nodes` and `/lifecycle_manager_navigation/manage_nodes`, sending `STARTUP` commands to bring all managed nodes into the `active` state.  
- **Initial pose publication**: publishes a `geometry_msgs/PoseWithCovarianceStamped` message on `/initialpose` immediately after localization is started, initializing AMCL without requiring manual input in RViz.  
- **Readiness signal**: publishes `/nav2_ready` once localization and navigation are active, ensuring deterministic synchronization with other nodes.  

### Parameters
- **initial_x** (double, default: 0.0): X coordinate of the initial pose in the map frame.  
- **initial_y** (double, default: 0.0): Y coordinate of the initial pose in the map frame.  
- **initial_yaw** (double, default: 0.0): Orientation (yaw, in radians) of the initial pose.  
- **service_wait_timeout_sec** (double, default: 10.0): Maximum time to wait for lifecycle services to become available.  
- **call_timeout_sec** (double, default: 10.0): Maximum time to wait for a lifecycle service call to complete.  

These parameters can be adjusted in the provided launch file (`global.launch.py`).

This design guarantees that the navigation stack is only activated once localization is stable, and that dependent nodes (GoalSelector and GoalSender) start working only after Nav2 is fully ready, resulting in a reliable and fully automated startup procedure.

## 🎯 Goal Selector Node
The **Goal Selector** node reads AprilTag detections and computes a navigation goal from the detected tags. It is designed to work with the `apriltag_node` that publishes tag frames on `/tf` and the `/detections` topic.

### Behavior and design
- **Input**: subscribes to `/detections` (`apriltag_msgs/AprilTagDetectionArray`) to learn which tag IDs are visible.  
- **Pose acquisition**: obtains each tag pose by performing a TF lookup from the configured `target_frame` (`map` or `odom`) to the tag frame (e.g. `tag36h11:1`).  
- **Goal calculation**: computes a goal as the midpoint between two specified tags. The result is a `geometry_msgs/PoseStamped` in the chosen target frame.  
- **Output**: exposes this goal through a **ROS 2 service**:
  ```
  /get_current_goal    (GetGoal.srv)
  ```
  whose interface is
  ```
  # Request
  int32 tag_id_1
  int32 tag_id_2
  ---
  # Response
  bool success
  string message
  geometry_msgs/PoseStamped goal
  ```
  **Why a service?** Previously, the node published goals periodically to a topic (`/goal_pose_raw`), which caused synchronization issues. The service-based approach ensures that the GoalSender always receives the most up-to-date goal deterministically, only when requested.

- **Synchronization**: the node subscribes to `/nav2_ready` and rejects service calls until Nav2 is fully operational, ensuring deterministic startup and avoiding invalid goals.

### Parameters
- **target_frame** (string, default: `"map"`): frame in which the goal is published and in which TF lookups are performed.  
- **tag_frame_prefix** (string, default: `"tag36h11:"`): prefix used to build the tag frame name from the integer tag ID (prefix + ID, e.g. `tag36h11:1`). Must match the names in `apriltag_node` configuration.  
- **tf_timeout_sec** (double, default: `0.3`): timeout used when waiting for the TF lookup.  

These parameters can be adjusted in the provided launch file (`global.launch.py`).

## 📡 Goal Sender Node
The **Goal Sender** node is responsible for requesting the latest navigation goal from the Goal Selector and sending it to Nav2 through the `navigate_to_pose` action. Goals are now exchanged deterministically via a service, eliminating the timing issues of topic-based communication.

The Goal Sender node is responsible for requesting the latest navigation goal from the Goal Selector and sending it to Nav2 through the `navigate_to_pose` action. Goals are exchanged deterministically via a service, eliminating the timing issues of topic-based communication. The node has been extended with **pause/resume logic** to support hybrid navigation scenarios (e.g., switching to a corridor controller).

### Behavior and design
- **Service client**: requests the current goal by calling `/get_goal` service.  
- **Action client**: connects to Nav2’s `navigate_to_pose` action server.  
- **Synchronization**: subscribes to `/nav2_ready` and remains inactive until Nav2 is fully operational.  
- **Pause/resume**: subscribes to `/corridor_active` topic.
  - When `true`, the node pauses and stops sending new goals.
  - When `false`, the node resumes and re-sends the last stored goal to Nav2.
- **Execution**: when active, calls the `GetGoal` service to retrieve the most recent `PoseStamped` goal computed by the Goal Selector, sends the goal to Nav2 using the `NavigateToPose` action, waits for the navigation result, and logs the outcome.  
- **Result handling**: processes action result codes (`SUCCEEDED`, `ABORTED`, `CANCELED`) and updates its internal state accordingly.
- **Shutdown**: after a successful navigation, the node can optionally call `rclcpp::shutdown()` to stop the system automatically.

This separation of responsibilities ensures that the Goal Selector only computes goals, while the Goal Sender handles navigation requests.

This service-based design ensures that: the Goal Sender always receives the most up-to-date goal, no periodic publishing is needed, timing issues between publishers and subscribers are eliminated, the node integrates reliably with the Nav2 action server.
### Parameters
- **tag_id_1** (int, default: `1`): ID of the first tag used to compute the goal.  
- **tag_id_2** (int, default: `10`): ID of the second tag used to compute the goal.  

These parameters can be adjusted in the provided launch file (`global.launch.py`).


# Detection_Lidar Node

- **Purpose**: Detects obstacles and wall segments using the onboard LDS LIDAR.  
- **Methods**:
  - **DBSCAN** clustering (from `scikit-learn`) to group LIDAR points into obstacles.  
  - **RANSAC** line fitting to robustly extract wall segments.  
- **Outputs**:
  - Publishes obstacles as `PoseArray` in the LIDAR frame and transformed into `/odom`.  
  - Publishes wall segments similarly.  

> **Dependency**: Install scikit-learn for DBSCAN  
> ```bash
> sudo apt update
> sudo apt install python3-sklearn
> ```

**Nota**: RANSAC algorithm, implemented for detecting walls, will be used for the (Extra) Corridor-Controller-Node without using Nav2.

# Corridor_Controller Node

The `Corridor_Controller` node implements a simple supervisory logic for navigation inside corridors:

- **Input:** It subscribes to the topic `table_detection/walls_odom`, which publishes wall segments detected by the LIDAR using RANSAC.
- **Corridor detection:** When exactly **two walls** are detected, the robot is assumed to be inside a corridor. In this state:
  - Nav2 is disabled.
  - The node publishes forward velocity commands (`cmd_vel`) to drive the robot straight along the corridor.
- **Corridor exit:** When **more than two walls** are detected, the corridor is considered finished. In this state:
  - The node stops manual control.
  - Nav2 is re-enabled to resume normal navigation.
- **Robustness:** To avoid oscillations due to noisy detections, the node requires the condition (two walls or more than two walls) to be stable for several consecutive cycles before switching states.
- **Outputs:**
  - `cmd_vel` → velocity commands during corridor traversal.
  - `nav2_enable` (Bool) → flag to enable/disable Nav2 depending on corridor state.

This design ensures that the robot moves forward reliably inside corridors without obstacles, and hands control back to Nav2 once the corridor ends.
