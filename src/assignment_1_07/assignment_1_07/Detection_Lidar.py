#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

import tf2_ros
import tf_transformations  # for quaternion → Euler (yaw) conversion if needed
from math import cos, sin, sqrt, atan2
import numpy as np
from sklearn.cluster import DBSCAN
import random


def se2_transform_points(points_xy, tx, ty, yaw):
    """Apply planar SE(2) transform to an array of 2D points."""
    cy = cos(yaw)
    sy = sin(yaw)
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    x_t = x * cy - y * sy + tx
    y_t = x * sy + y * cy + ty
    return np.stack([x_t, y_t], axis=1)


def ransac_lines(points, max_iterations=200, distance_thresh=0.05, min_inliers=30, angular_merge_thresh=0.15, distance_merge_thresh=0.20):
    """
    Robustly fit multiple line segments to 2D point set using RANSAC.
    - points: Nx2 array of (x, y)
    - max_iterations: RANSAC iterations per extraction
    - distance_thresh: inlier threshold (meters)
    - min_inliers: minimum points to accept a line
    - angular_merge_thresh: radians, merge lines with similar orientation
    - distance_merge_thresh: meters, merge lines with nearby centers

    Returns a list of dicts:
    [
      {
        'theta': orientation angle (radians),
        'p0': start point (x,y),
        'p1': end point (x,y),
        'center': midpoint (x,y),
        'length': segment length (meters),
        'inliers_idx': array of indices used
      }, ...
    ]
    """
    remaining = points.copy()
    all_indices = np.arange(points.shape[0])
    results = []

    def fit_line_two_points(a, b):
        # Parametric line from two points; store orientation (theta) and normal form
        # Orientation: angle of the line direction vector
        vx, vy = (b[0] - a[0]), (b[1] - a[1])
        theta = atan2(vy, vx)
        # Normal form: line defined by (n . x = d), where n is unit normal
        # Normal vector rotated by +90° from direction
        nx, ny = -sin(theta), cos(theta)
        d = nx * a[0] + ny * a[1]
        return theta, (nx, ny), d

    def point_line_distance(p, n, d):
        # Distance from point p to line (n . x = d)
        return abs(n[0] * p[0] + n[1] * p[1] - d)

    while remaining.shape[0] >= min_inliers:
        best_inliers = None
        best_model = None

        # RANSAC loop: sample 2 points, fit line, count inliers
        for _ in range(max_iterations):
            if remaining.shape[0] < 2:
                break
            i1, i2 = random.sample(range(remaining.shape[0]), 2)
            a = remaining[i1]
            b = remaining[i2]
            theta, n, d = fit_line_two_points(a, b)

            dists = np.abs(remaining @ np.array(n) - d)
            inliers_idx = np.where(dists < distance_thresh)[0]

            if inliers_idx.size >= min_inliers:
                # Track best by inlier count
                if best_inliers is None or inliers_idx.size > best_inliers.size:
                    best_inliers = inliers_idx
                    best_model = (theta, n, d)

        if best_inliers is None:
            # No more valid lines
            break

        # Construct segment endpoints using inliers projection along direction
        theta, n, d = best_model
        # Direction vector (unit)
        dirx, diry = cos(theta), sin(theta)

        inlier_pts = remaining[best_inliers]
        # Project inlier points onto the line to find segment endpoints (min/max projection)
        t_proj = inlier_pts @ np.array([dirx, diry])
        t_min_idx = np.argmin(t_proj)
        t_max_idx = np.argmax(t_proj)
        p0 = inlier_pts[t_min_idx]
        p1 = inlier_pts[t_max_idx]
        center = (p0 + p1) / 2.0
        length = float(np.linalg.norm(p1 - p0))

        # Map back to original indices (optional, here we keep in local)
        results.append({
            'theta': theta,
            'p0': p0,
            'p1': p1,
            'center': center,
            'length': length,
            'inliers_idx': best_inliers
        })

        # Remove inliers from remaining and continue extracting other lines
        mask = np.ones(remaining.shape[0], dtype=bool)
        mask[best_inliers] = False
        remaining = remaining[mask]

    # Merge similar lines to avoid duplicates
    merged = []
    for seg in results:
        merged_into_existing = False
        for m in merged:
            ang_diff = abs((seg['theta'] - m['theta'] + np.pi) % (2*np.pi) - np.pi)  # shortest angular diff
            cen_dist = np.linalg.norm(seg['center'] - m['center'])
            if ang_diff < angular_merge_thresh and cen_dist < distance_merge_thresh:
                # Extend the merged segment endpoints by checking all four endpoints
                candidates = np.vstack([m['p0'], m['p1'], seg['p0'], seg['p1']])
                # Project onto merged direction
                dirx, diry = cos(m['theta']), sin(m['theta'])
                t_proj = candidates @ np.array([dirx, diry])
                p0 = candidates[np.argmin(t_proj)]
                p1 = candidates[np.argmax(t_proj)]
                m['p0'] = p0
                m['p1'] = p1
                m['center'] = (p0 + p1) / 2.0
                m['length'] = float(np.linalg.norm(p1 - p0))
                merged_into_existing = True
                break
        if not merged_into_existing:
            merged.append(seg)

    return merged


class DetectionLidar(Node):
    def __init__(self):
        super().__init__('Detection_Lidar')

        # Subscribe to LaserScan
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )

        # Publishers: small obstacles and walls (source frame)
        self.pub_obstacles_src = self.create_publisher(PoseArray, 'table_detection/obstacles', 10)
        self.pub_walls_src = self.create_publisher(PoseArray, 'table_detection/walls', 10)
        # Publishers: transformed to odom frame
        self.pub_obstacles_odom = self.create_publisher(PoseArray, 'table_detection/obstacles_odom', 10)
        self.pub_walls_odom = self.create_publisher(PoseArray, 'table_detection/walls_odom', 10)
        

        # TF2 buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tunable parameters
        self.min_range = 0.05
        self.max_range = 6.0
        self.dbscan_eps = 0.14          # slightly tighter eps to separate legs from walls
        self.dbscan_min_samples = 4
        self.leg_diameter_min = 0.04
        self.leg_diameter_max = 0.25

        # RANSAC parameters
        self.ransac_max_iter = 200
        self.ransac_dist_thresh = 0.05
        self.ransac_min_inliers = 35     # require more points → robust walls
        self.ransac_ang_merge = 0.15     # ~8.6°
        self.ransac_dist_merge = 0.25

        self.get_logger().info("Detection_Lidar started, subscribed to /scan")

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
            self.get_logger().warn("No valid lidar points in this scan")
            return

        X = np.array(pts)
        self.get_logger().info(f"Received {len(X)} valid lidar points")

        # 1) Detect small obstacles via DBSCAN + size filter
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(X)
        labels = clustering.labels_

        obstacles_src = PoseArray()
        walls_src = PoseArray()
        obstacles_src.header.stamp = self.get_clock().now().to_msg()
        walls_src.header.stamp = self.get_clock().now().to_msg()
        obstacles_src.header.frame_id = msg.header.frame_id
        walls_src.header.frame_id = msg.header.frame_id

        obstacle_points = []  # Keep for potential fusion downstream

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
                # Small obstacle (e.g., table leg)
                pose = Pose()
                pose.position.x = float(cx)
                pose.position.y = float(cy)
                pose.orientation.w = 1.0
                obstacles_src.poses.append(pose)
                obstacle_points.append((cx, cy))
                self.get_logger().info(f"Obstacle detected at ({cx:.2f}, {cy:.2f}), diameter={diameter:.2f}")

        # 2) Detect wall segments via RANSAC on all points (robust line extraction)
        wall_segments = ransac_lines(
            X,
            max_iterations=self.ransac_max_iter,
            distance_thresh=self.ransac_dist_thresh,
            min_inliers=self.ransac_min_inliers,
            angular_merge_thresh=self.ransac_ang_merge,
            distance_merge_thresh=self.ransac_dist_merge
        )
        self.get_logger().info(f"RANSAC extracted {len(wall_segments)} wall segments")

        # Represent walls by segment midpoints and orientation (yaw in quaternion)
        for seg in wall_segments:
            center = seg['center']
            theta = seg['theta']  # line direction yaw
            # Quaternion from yaw
            qw = cos(theta / 2.0)
            qz = sin(theta / 2.0)  # in 2D, z-axis rotation
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            walls_src.poses.append(pose)

        # Publish source-frame results
        if obstacles_src.poses:
            self.pub_obstacles_src.publish(obstacles_src)
        if walls_src.poses:
            self.pub_walls_src.publish(walls_src)

        # 3) Transform both PoseArrays to odom frame (manual SE(2) for robustness)
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
                self.get_logger().info(f"Published {len(obstacles_odom.poses)} obstacles in odom")

            # Walls to odom (transform center only; preserve yaw)
            if walls_src.poses:
                src_xy_w = np.array([[p.position.x, p.position.y] for p in walls_src.poses])
                odom_xy_w = se2_transform_points(src_xy_w, tx, ty, yaw)
                walls_odom = PoseArray()
                walls_odom.header.stamp = self.get_clock().now().to_msg()
                walls_odom.header.frame_id = 'odom'
                for i, xy in enumerate(odom_xy_w):
                    pose = Pose()
                    pose.position.x = float(xy[0])
                    pose.position.y = float(xy[1])
                    # Compose yaw: yaw_odom = yaw_tf + yaw_src
                    # Approximate by adding z-rotation (2D); get source yaw from orientation
                    src_pose = walls_src.poses[i]
                    # Recover source yaw from quaternion (z,w used)
                    src_yaw = 2.0 * atan2(src_pose.orientation.z, src_pose.orientation.w)
                    yaw_odom = yaw + src_yaw
                    pose.orientation.z = float(sin(yaw_odom / 2.0))
                    pose.orientation.w = float(cos(yaw_odom / 2.0))
                    walls_odom.poses.append(pose)
                self.pub_walls_odom.publish(walls_odom)
                self.get_logger().info(f"Published {len(walls_odom.poses)} walls in odom")

        except Exception as e:
            frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().warn(f"TF transform to odom failed: {e}")
            self.get_logger().warn(f"Available TF frames:\n{frames}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
