#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool                   

import tf2_ros
import tf_transformations  
from math import cos, sin, atan2
import numpy as np
import random

# --- Helper Functions ---

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

    remaining = points.copy()
    results = []

    def fit_line_two_points(a, b):
        vx, vy = (b[0] - a[0]), (b[1] - a[1])
        theta = atan2(vy, vx)
        nx, ny = -sin(theta), cos(theta)
        d = nx * a[0] + ny * a[1]
        return theta, (nx, ny), d

    while remaining.shape[0] >= min_inliers:
        best_inliers = None
        best_model = None

        # RANSAC Sampling Loop
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
                if best_inliers is None or inliers_idx.size > best_inliers.size:
                    best_inliers = inliers_idx
                    best_model = (theta, n, d)

        if best_inliers is None:
            break

        # Extract segment endpoints from inliers
        theta, n, d = best_model
        dirx, diry = cos(theta), sin(theta)
        inlier_pts = remaining[best_inliers]
        
        # Project points onto the line direction to find start/end
        t_proj = inlier_pts @ np.array([dirx, diry])
        t_min_idx = np.argmin(t_proj)
        t_max_idx = np.argmax(t_proj)
        p0 = inlier_pts[t_min_idx]
        p1 = inlier_pts[t_max_idx]
        center = (p0 + p1) / 2.0
        length = float(np.linalg.norm(p1 - p0))

        results.append({
            'theta': theta,
            'p0': p0,
            'p1': p1,
            'center': center,
            'length': length,
            'inliers_idx': best_inliers
        })

        # Remove inliers from processing
        mask = np.ones(remaining.shape[0], dtype=bool)
        mask[best_inliers] = False
        remaining = remaining[mask]

    # Merge Logic: Connect segments that are likely the same wall broken by doors/noise
    merged = []
    for seg in results:
        merged_into_existing = False
        for m in merged:
            ang_diff = abs((seg['theta'] - m['theta'] + np.pi) % (2*np.pi) - np.pi)
            cen_dist = np.linalg.norm(seg['center'] - m['center'])
            
            # If segments are collinear and close to each other
            if ang_diff < angular_merge_thresh and cen_dist < distance_merge_thresh:
                candidates = np.vstack([m['p0'], m['p1'], seg['p0'], seg['p1']])
                dirx, diry = cos(m['theta']), sin(m['theta'])
                t_proj = candidates @ np.array([dirx, diry])
                p0 = candidates[np.argmin(t_proj)]
                p1 = candidates[np.argmax(t_proj)]
                
                # Update the existing merged line
                m['p0'] = p0
                m['p1'] = p1
                m['center'] = (p0 + p1) / 2.0
                m['length'] = float(np.linalg.norm(p1 - p0))
                merged_into_existing = True
                break
        if not merged_into_existing:
            merged.append(seg)

    return merged

# --- Node Class ---

class CorridorDetector(Node):
    def __init__(self):
        super().__init__('corridor_detector')

        # Subscription
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )

        # Publishers
        # Publishes True if corridor detected, False otherwise
        self.pub_is_corridor = self.create_publisher(Bool, '/corridor_active', 10)
        
        # Visualization publishers (Source frame and Odom frame)
        self.pub_walls_src = self.create_publisher(PoseArray, 'table_detection/walls', 10)
        self.pub_walls_odom = self.create_publisher(PoseArray, 'table_detection/walls_odom', 10)
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- PARAMETERS ---
        # RANSAC Parameters
        self.min_range = 0.05
        self.max_range = 10.0                # Increased range to see further down corridors
        self.ransac_max_iter = 200
        self.ransac_dist_thresh = 0.05
        self.ransac_min_inliers = 20
        self.ransac_ang_merge = 0.15
        self.ransac_dist_merge = 0.50 

        # Corridor Logic Parameters - - - - - - - - - - - - - - - - - - - - - - - -
        self.wall_length_threshold = 1      # [m] Minimum length for a wall to be considered "structural"
        self.min_long_walls = 2             # Minimum number of long walls required
        self.max_total_walls_allowed = 3    # STRICT FILTER: If we see 3 or more walls, it's likely a room, not a corridor.
                                            # We expect exactly 2 walls (Left + Right) for a corridor.
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        self.get_logger().info("Corridor_Detector started. Waiting for scan...")

    def lidar_callback(self, msg: LaserScan):
        # 1. Convert Scan to Cartesian
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

        # 2. Run RANSAC to find walls
        wall_segments = ransac_lines(
            X,
            max_iterations=self.ransac_max_iter,
            distance_thresh=self.ransac_dist_thresh,
            min_inliers=self.ransac_min_inliers,
            angular_merge_thresh=self.ransac_ang_merge,
            distance_merge_thresh=self.ransac_dist_merge
        )

        # 3. Analyze Walls for Corridor Logic
        total_walls_detected = len(wall_segments) # Total count of lines found by RANSAC
        long_walls_count = 0
        
        walls_src = PoseArray()
        walls_src.header.stamp = self.get_clock().now().to_msg()
        walls_src.header.frame_id = msg.header.frame_id

        for seg in wall_segments:
            # Check length
            if seg['length'] >= self.wall_length_threshold:
                # self.get_logger().info("Lunghezza muro: {:.2f} m".format(seg['length']))
                long_walls_count += 1
            
            # Visualization Prep
            center = seg['center']
            theta = seg['theta']
            qw = cos(theta / 2.0)
            qz = sin(theta / 2.0)
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            walls_src.poses.append(pose)

        # 4. Determine Corridor Status
        # Condition 1: We see at least 2 long walls (defining the corridor structure)
        has_structure = (long_walls_count >= self.min_long_walls)
        
        # Condition 2: We see fewer than 3 total walls (Clean corridor, no clutter/room furniture)
        is_clean_environment = (total_walls_detected < self.max_total_walls_allowed)

        # Final Decision: MUST meet both conditions
        is_corridor = has_structure and is_clean_environment
        
        # Publish Status
        bool_msg = Bool()
        bool_msg.data = is_corridor
        self.pub_is_corridor.publish(bool_msg)

        # Logging status for debugging
        if is_corridor:
             self.get_logger().info(f"[CORRIDOR FOUND] Total Walls: {total_walls_detected} | Long Walls: {long_walls_count}")
        else:
             # Optional: Log why it failed if you want to debug sensitivity
             pass 
             # self.get_logger().info(f"[NO CORRIDOR] Total: {total_walls_detected}, Long: {long_walls_count}")

        # 5. Publish Visualization (Walls)
        if walls_src.poses:
            self.pub_walls_src.publish(walls_src)

        # 6. Transform and Publish to Odom Frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', walls_src.header.frame_id, rclpy.time.Time()
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

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
                    
                    # Transform Orientation
                    src_pose = walls_src.poses[i]
                    src_yaw = 2.0 * atan2(src_pose.orientation.z, src_pose.orientation.w)
                    yaw_odom = yaw + src_yaw
                    
                    pose.orientation.z = float(sin(yaw_odom / 2.0))
                    pose.orientation.w = float(cos(yaw_odom / 2.0))
                    walls_odom.poses.append(pose)
                    
                self.pub_walls_odom.publish(walls_odom)

        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CorridorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()