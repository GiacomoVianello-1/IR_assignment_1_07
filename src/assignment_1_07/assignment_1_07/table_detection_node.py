#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        self.sub_obstacles = self.create_subscription(
            PoseArray, '/obstacles_odom', self.obstacles_callback, 10
        )
        self.sub_camera = self.create_subscription(
            PoseArray, '/camera_odom', self.camera_callback, 10
        )

        self.last_obstacles = None
        self.last_camera = None

        # Association radius (XY) for matching camera to LIDAR
        self.association_radius = 0.45

        self.get_logger().info("TableDetectionNode started (robust association + log)")

    def obstacles_callback(self, msg: PoseArray):
        # Defensive: ensure odom frame
        if msg.header.frame_id and msg.header.frame_id != 'odom':
            self.get_logger().warn(f"Received obstacles in frame '{msg.header.frame_id}', expected 'odom'")
        self.last_obstacles = msg
        self.try_fuse()

    def camera_callback(self, msg: PoseArray):
        if msg.header.frame_id and msg.header.frame_id != 'odom':
            self.get_logger().warn(f"Received camera obstacles in frame '{msg.header.frame_id}', expected 'odom'")
        self.last_camera = msg
        self.try_fuse()

    def try_fuse(self):
        # Proceed if at least one source present
        if self.last_obstacles is None and self.last_camera is None:
            return

        # Extract arrays
        lidar_pts = []
        if self.last_obstacles and self.last_obstacles.poses:
            for p in self.last_obstacles.poses:
                lidar_pts.append([p.position.x, p.position.y, p.position.z])
        camera_pts = []
        if self.last_camera and self.last_camera.poses:
            for p in self.last_camera.poses:
                camera_pts.append([p.position.x, p.position.y, p.position.z])

        lidar_pts = np.array(lidar_pts, dtype=np.float32)
        camera_pts = np.array(camera_pts, dtype=np.float32)

        # Logging counts before association
        self.get_logger().info(f"LIDAR obstacles considered: {len(lidar_pts)} | Camera obstacles considered: {len(camera_pts)}")

        if lidar_pts.size == 0 and camera_pts.size == 0:
            return

        fused = []
        matched_camera = set()
        matched_lidar = set()

        # Associate only if both sources present
        if lidar_pts.size > 0 and camera_pts.size > 0:
            for i, lp in enumerate(lidar_pts):
                # Nearest neighbor in XY
                dists = np.linalg.norm(camera_pts[:, :2] - lp[:2], axis=1)
                j = int(np.argmin(dists))
                if dists[j] <= self.association_radius:
                    matched_lidar.add(i)
                    matched_camera.add(j)
                    fused_pos = 0.5 * (lp + camera_pts[j])
                    fused_pos[2] = 0.0  # ground-level fusion
                    fused.append(fused_pos)

        unmatched_lidar = [lidar_pts[i] for i in range(len(lidar_pts)) if i not in matched_lidar]
        unmatched_camera = [camera_pts[j] for j in range(len(camera_pts)) if j not in matched_camera]

        # Summary
        self.get_logger().info(f"Fused matches: {len(fused)} | Unmatched LIDAR: {len(unmatched_lidar)} | Unmatched Camera: {len(unmatched_camera)}")

        # Detailed logs
        for k, c in enumerate(fused, start=1):
            self.get_logger().info(f"[FUSED] Obstacle {k}: x={c[0]:.2f}, y={c[1]:.2f}, z={c[2]:.2f}")
        for k, lp in enumerate(unmatched_lidar, start=1):
            self.get_logger().info(f"[LIDAR only] {k}: x={lp[0]:.2f}, y={lp[1]:.2f}, z={lp[2]:.2f}")
        for k, cp in enumerate(unmatched_camera, start=1):
            self.get_logger().info(f"[CAMERA only] {k}: x={cp[0]:.2f}, y={cp[1]:.2f}, z={cp[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TableDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
