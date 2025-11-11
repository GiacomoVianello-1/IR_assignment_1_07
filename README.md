# 🤖 Intelligent Robotics — Assignment 1  
## Group 07  
- Giacomo Vianello (ID: 2140028)  
- Salvatore Ferracane (ID: 2154255)  

## 📘 Project Overview  

This repository contains our solution to [Assignment 1](Assignment_1.pdf) for the *Intelligent Robotics* course at the University of Padua. Our implementation builds upon the base repository [`ir_2526`](https://github.com/PieroSimonet/ir_2526.git), which is included under the `src/` directory.

The project is structured as a ROS 2 workspace and includes all necessary components to run, test, and extend our assignment solution.

## 📷 Apriltags and Camera Connections

To enable AprilTag detection, launch the apriltag_node executable with the appropriate topic remappings and configuration file:
```sh
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/rgb_camera/image \
  -r camera_info:=/rgb_camera/camera_info \
  --params-file $(ros2 pkg prefix apriltag_ros)/share/apriltag_ros/cfg/tags_36h11.yaml
```
**Note**: Update the configuration file (`tags_36h11.yaml`) to match the actual tag size used in simulation. In this setup, the tags measure **0.05 m x 0.05 m**.

**TODO**: *Automatize this process in a proper launch file*.

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