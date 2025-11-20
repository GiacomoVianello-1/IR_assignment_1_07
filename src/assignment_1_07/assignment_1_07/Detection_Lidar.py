#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

import tf2_ros
import tf_transformations  
from math import cos, sin, atan2
import numpy as np
from sklearn.cluster import DBSCAN

# Transform points using SE(2) transformation
def se2_transform_points(points_xy, tx, ty, yaw):
    """Apply planar SE(2) transform to an array of 2D points."""
    cy = cos(yaw)
    sy = sin(yaw)
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    x_t = x * cy - y * sy + tx
    y_t = x * sy + y * cy + ty
    return np.stack([x_t, y_t], axis=1)

class DetectionLidar(Node):
    def __init__(self):
        super().__init__('Detection_Lidar')

        # Subscribe to LaserScan
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )

        # Publishers: small obstacles (source frame and odom frame)
        self.pub_obstacles_src = self.create_publisher(PoseArray, 'table_detection/obstacles', 10)
        self.pub_obstacles_odom = self.create_publisher(PoseArray, 'table_detection/obstacles_odom', 10)

        # TF2 buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tunable parameters for DBSCAN
        self.min_range = 0.05
        self.max_range = 6.0
        self.dbscan_eps = 0.14          
        self.dbscan_min_samples = 4
        self.leg_diameter_min = 0.015   # minimum obstacle size (diameter) in meters
        self.leg_diameter_max = 0.25    # maximum obstacle size (diameter) in meters

        self.get_logger().info("Detection_Lidar (DBSCAN) started, subscribed to /scan")

    def lidar_callback(self, msg: LaserScan):
        # Convert LaserScan polar data to Cartesian points in sensor frame
        pts = []
        for i, r in enumerate(msg.ranges):
            if r == float('inf') or r != r:
                continue
            if r < self.min_range or r > self.max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * cos(angle)
            y = r * sin(angle)
            pts.append((x, y))
        
        if not pts:
            return

        X = np.array(pts)

        # 1) Detect obstacles via DBSCAN + size filter
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(X)
        labels = clustering.labels_

        obstacles_src = PoseArray()
        obstacles_src.header.stamp = self.get_clock().now().to_msg()
        obstacles_src.header.frame_id = msg.header.frame_id

        for label in set(labels):
            if label == -1:
                continue
            cluster_points = X[labels == label]
            if cluster_points.shape[0] < self.dbscan_min_samples:
                continue

            cx, cy = cluster_points.mean(axis=0)
            dists = np.linalg.norm(cluster_points - np.array([cx, cy]), axis=1)
            diameter = (float(np.max(dists)) * 2.0) if dists.size > 0 else 0.0

            if self.leg_diameter_min < diameter < self.leg_diameter_max:
                # Obstacle Detected
                pose = Pose()
                pose.position.x = float(cx)
                pose.position.y = float(cy)
                pose.orientation.w = 1.0
                obstacles_src.poses.append(pose)
                self.get_logger().info(f"Obstacle detected at ({cx:.2f}, {cy:.2f}), diameter={diameter:.2f}m")

        # Publish source-frame results
        if obstacles_src.poses:
            self.pub_obstacles_src.publish(obstacles_src)

        # 2) Transform PoseArray to odom frame 
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', obstacles_src.header.frame_id, rclpy.time.Time()
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            # Obstacles to odom
            if obstacles_src.poses:
                src_xy = np.array([[p.position.x, p.position.y] for p in obstacles_src.poses])
                odom_xy = se2_transform_points(src_xy, tx, ty, yaw)
                obstacles_odom = PoseArray()
                obstacles_odom.header.stamp = self.get_clock().now().to_msg()
                obstacles_odom.header.frame_id = 'odom'
                for xy in odom_xy:
                    pose = Pose()
                    pose.position.x = float(xy[0])
                    pose.position.y = float(xy[1])
                    pose.orientation.w = 1.0
                    obstacles_odom.poses.append(pose)
                self.pub_obstacles_odom.publish(obstacles_odom)

        except Exception as e:
            # Log only if there are obstacles to transform
            if obstacles_src.poses:
                self.get_logger().warn(f"TF transform to odom failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()