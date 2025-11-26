# 🤖 Intelligent Robotics — Assignment 1  
### Group 07 Members 
- Giacomo Vianello (ID: 2140028, [mail](mailto:giacomo.vianello.2@studenti.unipd.it)) 
- Salvatore Ferracane (ID: 2154255, [mail](mailto:salvatore.ferracane@studenti.unipd.it))

This repository contains our solution to [Assignment 1](Assignment_1.pdf) for the *Intelligent Robotics* course at the University of Padua. Our implementation builds upon the base repository [`ir_2526`](https://github.com/PieroSimonet/ir_2526.git), which is included under the `src/` directory.
The project is structured as a ROS 2 workspace and includes all necessary components to run, test, and extend our assignment solution.


## 🏃‍♂️‍➡️ Run the Project

### 1. Preliminaries
Clone this repository onto your local machine. In addition to the standard ROS 2 packages suggested at the beginning of the course, make sure the following dependencies are installed:
```
sudo apt install ros-${ROS_DISTRO}-tf-transformations
sudo apt install python3-sklearn
```
It is also recommended to update your system before proceeding:
```
sudo apt update && sudo apt upgrade
```

### 2. Import Required Package
Inside the cloned repository, add the `ir_2526` package to the source tree to ensure proper linking during compilation:
```
cd src
git clone https://github.com/PieroSimonet/ir_2526.git
```

### 3. Build the Workspace
Compile the entire workspace and source the environment:
```
colcon build
source install/setup.bash
```
### 4. Launch the Project
We provide a structured and configurable launch file named `global.launch.py`. You can start the project with:
```
ros2 launch assignment_1_07 global.launch.py
```

# 📘 Project Overview  

To provide a clear understanding of our architecture, we include a comprehensive UML diagram that illustrates the entire solution. The diagram highlights:
- All the **nodes** involved (both those we developed and those provided).
- The communication methods between them, including **topics**, **services**, and **actions**.

In the following sections of this README, we provide a detailed analysis of the behavior and functionality of each node. 

![alt text](Assignment1_UML.svg)


## 📂 Workspace Structure

This project uses a **hybrid ROS2 workspace** to develop nodes in both **C++** and **Python**. In particular
- All **Python nodes** are placed inside the `/assignment_1_07` folder (executables).  
- All **C++ nodes** are placed inside the `/src` folder.   

The package structure, together with the `CMakeLists.txt` and `package.xml`, has been adapted following the guidelines from this [tutorial](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/).

```
assignment_1_07/
├── assignment_1_07
│   ├── Corridor_Controller.py
│   ├── corridor_detector.py
│   ├── Detection_Lidar.py
│   └── __init__.py
├── CMakeLists.txt
├── config
│   └── apriltag_params.yaml
├── include
│   └── assignment_1_07
├── launch
│   └── global.launch.py
├── LICENSE
├── package.xml
├── src
│   ├── cancel_nav2_goal.cpp
│   ├── goal_selector.cpp
│   ├── goal_sender.cpp
│   └── nav2_orchestrator.cpp
└── srv
    └── GetGoal.srv
```

## 🚀 Launch File
The `global.launch.py` file orchestrates the entire assignment setup, ensuring that all required nodes and configurations are started with a single command:
```
source install/setup.bash 
ros2 launch assignment_1_07 global.launch.py
```

This launch file coordinates the following components:

- **Base launch file inclusion**: integrates `ir_launch/assignment_1.launch.py` from the provided repository.
- **AprilTag Detection Node**: starts with the correct topic remappings and parameter configuration.
- **Nav2Orchestrator Node**: initializes the localization and navigation stacks, publishes the initial pose to AMCL, and signals readiness.
- **Goal Selector Node**: computes navigation goals using TF lookups of detected tags and a custom service interface.
- **Goal Sender Node**: queries the `/get_current_goal` service and forwards the result to the Nav2 `navigate_to_pose` action.
- **Cancel Nav2 Goal Node**: cancels the current navigation goal whenever the robot detects that it has entered a corridor.
- **Corridor Detector Node**: applies a RANSAC algorithm to determine whether the robot is inside a corridor.
- **Corridor Controller Node**: acts as a PD controller to drive the TurtleBot through the corridor.
- **Detection Lidar Node**: identifies tables within the simulation environment.

With this setup, a single command brings up the entire system, fully configured and ready for testing or demonstration. All parameters are clearly defined within the launch file, making them easy to understand and adapt to different scenarios. 


## 📷 Apriltags Ros Node (and Camera Linking)

We rely on the external package `apriltag_ros`, which uses the AprilTag library to detect tags in camera images and publish their pose, ID, and metadata.
Normally, enabling AprilTag detection requires adjusting the default configuration file (`tags_36h11.yaml`) inside the `apriltag_ros` package, and then running the node with the correct topic remappings. In our case, this could be done with 
```sh
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/rgb_camera/image \
  -r camera_info:=/rgb_camera/camera_info \
  --params-file $(ros2 pkg prefix apriltag_ros)/share/apriltag_ros/cfg/tags_36h11.yaml
```
However, *instead of modifying the default configuration file in the `apriltag_ros` package*, we integrated these adjustments and remappings in our own custom launch file (`global.launch.py`). In this way:
- The launch file automatically loads the configuration from `assignment_1_07/config/apriltag_params.yaml`, which specifies the actual tag size (**0.05 m × 0.05 m**) and other relevant parameters.
- If the AprilTag setup changes in the future (e.g., different tag family or size), we only need to update *our* YAML file without touching the external package.
- The launch file ensures that the AprilTag node runs alongside the rest of our system (goal selector, orchestrator, etc.), so everything is initialized consistently.

### Published topics
Once running, the node provides:
- `/tf` - publishes transforms (`tf2_msgs/msg/TFMessage`) for each detected tag.
- `/detections` - publishes detection results (`apriltag_msgs/msg/AprilTagDetectionArray`) including tag IDs, poses, and metadata.

### Visualization Tips
To confirm that the node is working:
1. Launch `RViz2`
2. Select `odom` as fixed frame.
3. Add a **Display -> Image** and select `/rgb_camera/image` to see the camera feed.
4. Add a **Display -> TF** to visualize the coordinate frames of detected tags (e.g., `tag36h11:0`).

Now we should see the two apriltag frames (`tag36h11:1`, and `tag36h11:10`) wrt the `odom` frame. 

**REMARK:** the tables (cylindrical objects in Gazebo) are not in field of view of the Camera. Hence, we will need to localize them using the on-board TurlteBot LiDAR sensor (see below).


## 🎼 Nav2 Orchestrator Node
The **Nav2Orchestrator** node automates the initialization of the full navigation stack by interacting directly with the lifecycle managers of both localization and navigation. Instead of requiring manual service calls or user input in RViz, this node ensures that all components are correctly configured and activated in sequence, and publishes a readiness signal for the rest of the system.

### Behavior and design

Our design is motivated by a key observation: after activating the localization stack via `/lifecycle_manager_localization/manage_nodes`, the navigation lifecycle manager will reject a `STARTUP` request (`success=false`) if the system has not yet received a valid initial pose estimate. To address this, the orchestrator executes the following four steps:

1. Start the localization lifecycle manager.  
2. Publish the initial pose to AMCL (with a configurable covariance).  
3. Wait briefly for AMCL to process the initial pose.  
4. Start the navigation lifecycle manager.  

Once both lifecycle managers confirm success, the orchestrator publishes a `std_msgs/Bool` message on the `/nav2_ready` topic. This readiness signal ensures that the **GoalSelector** and **GoalSender** nodes activate their logic only after Nav2 is fully operational. By gating their execution on this condition, the system guarantees that the navigation stack comes online only once localization is stable. As a result, dependent nodes begin functioning at the right moment, delivering a reliable and fully automated startup sequence.

### Features
- **Lifecycle management**: connects to `/lifecycle_manager_localization/manage_nodes` and `/lifecycle_manager_navigation/manage_nodes`, sending `STARTUP` commands to bring all managed nodes into the `active` state.  
- **Initial pose publication**: publishes a `geometry_msgs/PoseWithCovarianceStamped` message on `/initialpose` immediately after localization is started, initializing AMCL without requiring manual input in RViz.  
- **Readiness signal**: publishes `/nav2_ready` once localization and navigation are active, ensuring deterministic synchronization with other nodes.  

### Parameters
- `initial_x`: X coordinate of the initial pose in the map frame.  
- `initial_y`: Y coordinate of the initial pose in the map frame.  
- `initial_yaw`: Orientation (yaw, in radians) of the initial pose. 
- `covariance_x`: Initial covariance in the X-coordinate estimate.
- `covariance_y`: Initial covariance in the Y-coordinate estimate.
- `covariance_yaw`: Initial covariance in the heading angle estimate. 
- `service_wait_timeout_sec`: Maximum time to wait for lifecycle services to become available.  
- `call_timeout_sec`: Maximum time to wait for a lifecycle service call to complete.  

These parameters can be adjusted at will in the provided launch file (`global.launch.py`).

## 🎯 Goal Selector Node
The **Goal Selector** node reads AprilTag detections and computes a navigation goal from the detected tags. It is designed to work with the `apriltag_ros` node that publishes tag frames on `/tf` and the `/detections` topic.

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

This service-based design ensures that the GoalSender always receives the most up-to-date goal, no periodic publishing is needed, timing issues between publishers and subscribers are eliminated, the node integrates reliably with the Nav2 action server.
### Parameters
- **tag_id_1** (int, default: `1`): ID of the first tag used to compute the goal.  
- **tag_id_2** (int, default: `10`): ID of the second tag used to compute the goal.  

These parameters can be adjusted in the provided launch file (`global.launch.py`).

## 🛑 Cancel Nav2 Goal Node

The CancelNav2Goal node is responsible for canceling all active navigation goals in Nav2 when the `corridor_detector` node senses the walls in the corridor. It integrates with the hybrid navigation architecture by listening to the `/corridor_active` topic and triggering cancellation only when corridor mode is active.

### Behavior and Design
- **Action client**: connects to Nav2’s `navigate_to_pose` action server.
- **Subscription**: listens to `/corridor_active` (`std_msgs/Bool`).
  - When `true`, the node sends a cancellation request to Nav2, stopping the current navigation goal.
  - When `false`, no cancellation is sent (Nav2 continues normally).
- **Integration**: works alongside the Goal Sender and Corridor Controller.
  - Goal Sender pauses when `/corridor_active=true`.
  - CancelNav2Goal ensures Nav2’s current goal is canceled at the same time.

## 🔎 Detection Lidar Node

*This node requires to install*
```
sudo apt install python3-sklearn
```

The `Detection_Lidar` node performs obstacle detection from raw LiDAR scans using geometric preprocessing, DBSCAN clustering, and temporal stabilization. It outputs the detected obstacle centers in the `odom` frame and produces a stabilized estimate of three obstacles when their positions remain sufficiently stable.

### Behavior and design

1. **Range filtering and Cartesian projection**: LiDAR polar data are converted into $(x,y)$ points in the sensor frame after rejecting invalid or out-of-range values.

2. **DBSCAN clustering**: The incoming point cloud is segmented into clusters with DBSCAN (`eps`, `min_samples`). Each cluster centroid is considered a potential obstacle.

3. **Size-based obstacle filtering**: The cluster diameter is computed. Only clusters whose diameter falls within a physically plausible interval are accepted as obstacles.

4. **Coordinate transformation (TF)**: Valid obstacle centroids in the sensor frame are transformed into the `odom` frame using SE(2) geometry:
   - translation (`tx`, `ty`)
   - yaw angle extracted from the TF quaternion

5. **Publishing**: All valid obstacles are published as a `PoseArray` on:
   - `/table_detection/obstacles_odom`

6. **Temporal stabilization (ObstacleTracker)**: The node implements a stabilization layer to track **exactly three obstacles**:
   - positions are sorted for consistent ordering  
   - variation over time is measured  
   - stable positions are updated only when variation exceeds a threshold  
   - when all three obstacle positions remain stable for consecutive frames, the list is printed once

### Stabilization Logic Summary
The `ObstacleTracker` class:
- tracks three obstacle points across time  
- updates the internal model only when changes exceed a spatial threshold  
- detects when the obstacle configuration becomes stable  
- prints the stabilized coordinates exactly once  
- resets if fewer than three obstacles are detected  

This prevents oscillations due to clustering noise and ensures a clean, reliable estimate of the three tracked obstacle positions.

**REMARK**: RANSAC algorithm, implemented for detecting walls, will be used for the (Extra) Corridor-Controller-Node without using Nav2.


## 🧠 Corridor Detector Node

*This node requires to install*
```
sudo apt install ros-${ROS_DISTRO}-tf-transformations
```

The `Corridor_Detector` node performs geometric wall detection using LIDAR data and determines whether the robot is currently inside a corridor. The node extracts wall segments from raw LaserScan measurements and applies a temporal stability filter to avoid false detections caused by noise.

### Behavior and design
1. **Segmentation and filtering**: The node extracts line segments from the LaserScan. Only segments satisfying minimum length and geometric consistency constraints are accepted as valid walls.  
2. **Coordinate transformation**: Detected walls are mapped from the sensor frame into the odometry frame using `tf_transformations` and an $\mathbb{SE}(2)$ rigid-body transform.
3. **Corridor identification**:  
   When **exactly two** stable walls are detected, the robot is considered inside a corridor.  
   When **more than two** or **fewer than two** walls are observed, the corridor is considered inactive.
4. **Temporal stabilization**: To prevent oscillatory state transitions caused by noisy detections, the node uses consecutive positive/negative counters.  
    - Corridor is activated only after $N$ consecutive frames confirming two walls.  
    - Corridor is deactivated only after $M$ consecutive frames confirming loss of corridor structure.
5. **Outputs**
    - `/table_detection/walls_odom` (PoseArray): Stabilized wall segments expressed in the odometry frame.
    - `/corridor_active` (Bool):  
        - `True` -> The robot is inside a corridor  
        - `False` -> Corridor condition not satisfied


## 🚨 Corridor Controller Node

The `Corridor_Controller` node implements a PD steering law that drives the robot forward whenever the corridor detector reports the presence of two approximately parallel walls. The controller uses the geometric information from the detected wall endpoints to maintain lateral centering and heading alignment. The control action consists of a constant forward velocity combined with a steering command computed as a proportional-derivative (PD) control: the lateral deviation and heading error provide the proportional components, while their rates of change contribute to the derivative components, damping oscillations and improving stability.

### Behavior and design
1. **Activation**: When `/corridor_active` becomes `True`, the controller begins driving forward at a constant speed. When the flag becomes `False`, the robot is stopped immediately.
2. **Wall projection into base frame**: Wall endpoints are transformed into `base_link` coordinates via TF2. If transformation fails or walls are temporarily missing, the controller applies a fallback behavior.
3. **Fallback forward motion**: If fewer than two walls are available but the corridor flag is still active, the controller publishes straight motion with zero angular velocity to maintain forward progress.

**Feedback**
1. **Lateral error computation**: The midpoint of the two detected wall endpoints is projected into the robot frame. Its `y` coordinate represents the lateral displacement relative to the corridor centerline.
2. **Heading error computation**: The orientation of the corridor is estimated from the direction of the segment connecting the two wall points. The angle is normalized to the interval $[- \pi/2, \pi/2]$.
3. **Deadband filtering**: Small lateral errors within a configurable threshold are set to zero. This prevents oscillations due to microscopic deviations and ensures stable straight motion.

**Control synthesis**  
   The controller applies:
   - Angle (heading) correction: Proportional to the heading error plus derivative term for angular damping.
   - Centering (lateral) correction: Proportional to lateral deviation plus derivative term for damping lateral oscillations.
