#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

import tf2_ros
import tf_transformations  
from math import cos, sin
import numpy as np
from sklearn.cluster import DBSCAN
from typing import Optional, List, Tuple

# --- Logic for Position Stabilization and Tracking ---
class ObstacleTracker:
    """Manages the stabilization and tracking logic for the 3 obstacle positions."""
    def __init__(self, threshold: float = 0.05):
        self.POSITION_THRESHOLD = threshold # Variation threshold (in meters)
        # List of stabilized obstacle positions: [(x1, y1), (x2, y2), (x3, y3)]
        self.stabilized_positions: List[Tuple[float, float]] = [] 
        # Indicates if the stabilized positions list has been successfully printed
        self.printed_once = False 

    def update_and_check(self, current_poses: List[Pose]) -> Optional[List[Tuple[float, float]]]:
        """
        Updates positions and checks if they are stable.
        Returns the list of positions if stabilized and not yet printed, otherwise None.
        """
        # STEP 1: Group current positions into a convenient format (XY only)
        current_xy = [(p.position.x, p.position.y) for p in current_poses]

        # Must detect exactly 3 obstacles to proceed with stabilization
        if len(current_xy) != 3:
            # Reset print state if tracking is lost
            self.printed_once = False
            return None

        # STEP 2: Sort the positions to ensure consistency (necessary for tracking)
        # Sort by X, then by Y. This attempts to match the same obstacle over time.
        current_xy.sort(key=lambda p: (p[0], p[1]))

        if not self.stabilized_positions:
            # First detection: initialize the stabilized positions
            self.stabilized_positions = current_xy
            return None

        # STEP 3: Compare current positions with stabilized positions
        total_delta = 0.0
        new_stabilized_positions: List[Tuple[float, float]] = []

        for current_p, stable_p in zip(current_xy, self.stabilized_positions):
            dx = current_p[0] - stable_p[0]
            dy = current_p[1] - stable_p[1]
            # Euclidean distance between current and last stable position
            delta = np.sqrt(dx**2 + dy**2)
            total_delta += delta

            if delta > self.POSITION_THRESHOLD:
                # Significant variation: update the stable position
                new_stabilized_positions.append(current_p)
            else:
                # Insignificant variation: maintain the previous stable position
                new_stabilized_positions.append(stable_p)
        
        # Update the internal state with the new stabilized/maintained positions
        self.stabilized_positions = new_stabilized_positions

        # STEP 4: Print the result only if stabilized AND not yet printed
        # Considered stable if the average variation is below half the threshold
        average_delta = total_delta / 3.0
        
        if average_delta < (self.POSITION_THRESHOLD / 2.0) and not self.printed_once:
            self.printed_once = True
            # Return positions for printing (format [(x1,y1), (x2,y2), (x3,y3)])
            return self.stabilized_positions
        
        return None


# Transform points using SE(2) transformation
def se2_transform_points(points_xy: np.ndarray, tx: float, ty: float, yaw: float) -> np.ndarray:
    """Apply planar SE(2) transform to an array of 2D points."""
    cy = cos(yaw)
    sy = sin(yaw)
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    x_t = x * cy - y * sy + tx
    y_t = x * sy + y * cy + ty
    return np.stack([x_t, y_t], axis=1)

# --- Node Implementation ---

class DetectionLidar(Node):
    def __init__(self):
        super().__init__('Detection_Lidar')

        # Subscribe to LaserScan
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )

        # Publisher: obstacles in the odom frame (the only required topic)
        self.pub_obstacles_odom = self.create_publisher(PoseArray, 'table_detection/obstacles_odom', 10)

        # TF2 buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Tunable parameters for DBSCAN and size filter
        self.min_range = 0.05
        self.max_range = 6.0
        self.dbscan_eps = 0.14          
        self.dbscan_min_samples = 4
        self.leg_diameter_min = 0.01    # minimum obstacle size (diameter) in meters
        self.leg_diameter_max = 0.22    # maximum obstacle size (diameter) in meters

        # Initialize the stabilization tracker
        self.tracker = ObstacleTracker(threshold=0.05) 

        self.get_logger().info("Detection_Lidar (DBSCAN) started, subscribing to /scan and publishing to /table_detection/obstacles_odom")

    def lidar_callback(self, msg: LaserScan):
        # 1) Convert LaserScan polar data to Cartesian points in sensor frame
        pts: List[Tuple[float, float]] = []
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

        # 2) Detect obstacles via DBSCAN + size filter (in sensor frame)
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(X)
        labels = clustering.labels_

        # Use a temporary list to hold obstacle poses in the sensor frame
        obstacles_src_poses: List[Pose] = []
        src_frame_id = msg.header.frame_id

        for label in set(labels):
            if label == -1: # Noise
                continue
            
            cluster_points = X[labels == label]
            if cluster_points.shape[0] < self.dbscan_min_samples:
                continue

            cx, cy = cluster_points.mean(axis=0)
            dists = np.linalg.norm(cluster_points - np.array([cx, cy]), axis=1)
            diameter = (float(np.max(dists)) * 2.0) if dists.size > 0 else 0.0

            if self.leg_diameter_min < diameter < self.leg_diameter_max:
                # Obstacle Detected (in sensor frame)
                pose = Pose()
                pose.position.x = float(cx)
                pose.position.y = float(cy)
                pose.orientation.w = 1.0
                obstacles_src_poses.append(pose)


        # 3) Transform Poses to odom frame and publish
        odom_poses: List[Pose] = []
        
        if not obstacles_src_poses:
            return # No obstacles detected, nothing to publish/track

        try:
            # Look up the transform from the sensor frame to 'odom'
            transform = self.tf_buffer.lookup_transform(
                'odom', src_frame_id, rclpy.time.Time()
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            # Transform obstacle coordinates
            src_xy = np.array([[p.position.x, p.position.y] for p in obstacles_src_poses])
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
                odom_poses.append(pose) # Save for the tracker
            
            # Publish the results in the odom frame (the ONLY required output topic)
            self.pub_obstacles_odom.publish(obstacles_odom)

        except Exception as e:
            # Log only if there are obstacles to transform (avoids continuous TF logs)
            self.get_logger().warn(f"TF transformation to odom failed: {e}")
            return
        
        # 4) Stabilization and Logging Logic
        # Use poses in the 'odom' frame
        stabilized_xy = self.tracker.update_and_check(odom_poses)

        if stabilized_xy is not None:
            # The condition has been met: 3 obstacles detected and stabilized.
            log_output = "✅ Obstacle Positions ('odom' frame):\n"
            for i, (x, y) in enumerate(stabilized_xy):
                log_output += f"  Obstacle {i+1}: X={x:.3f}, Y={y:.3f}\n"
            
            # Clean and clear print, published once
            self.get_logger().info(log_output)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()