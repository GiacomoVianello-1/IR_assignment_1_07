#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros
import tf_transformations
import cv2
from cv_bridge import CvBridge
import numpy as np

class DetectionCamera(Node):
    def __init__(self):
        super().__init__('Detection_Camera')

        # Subscriptions: your actual topics
        self.sub_rgb = self.create_subscription(
            Image, '/rgb_camera/image', self.rgb_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/world/default/model/external_camera/link/link/sensor/depth_camera/depth_image',
            self.depth_callback, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, '/rgb_camera/camera_info', self.info_callback, 10
        )

        # Publisher: obstacles transformed into odom
        self.pub_camera_odom = self.create_publisher(PoseArray, '/camera_odom', 10)

        # TF2 buffer/listener with longer cache to reduce extrapolation issues
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Bridge and intrinsics
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        # Cache last depth frame
        self.last_depth = None

        # Config
        self.min_contour_area = 600
        self.min_depth = 0.2
        self.max_depth = 6.0
        self.project_to_ground = True  # project camera obstacles to z=0 for lidar fusion

        # Frame names: prefer TurtleBot camera frames
        self.tb_rgb_optical = 'camera_rgb_optical_frame'
        self.tb_depth_optical = 'camera_depth_optical_frame'

        self.get_logger().info("Detection_Camera started (RGB+Depth, robust TF)")

    def info_callback(self, msg: CameraInfo):
        # Intrinsics from K matrix
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info("Camera intrinsics updated")

    def depth_callback(self, msg: Image):
        try:
            # Convert depth to float meters
            if msg.encoding in ['32FC1', '32FC']:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding in ['16UC1', 'mono16']:
                d16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth = d16.astype(np.float32) / 1000.0
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
                return
            self.last_depth = (depth, msg.header)
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def rgb_callback(self, msg: Image):
        # Require intrinsics and recent depth
        if self.fx is None or self.last_depth is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB conversion failed: {e}")
            return

        depth_img, depth_header = self.last_depth

        # Ensure images are aligned (same resolution)
        if depth_img.shape[:2] != frame.shape[:2]:
            self.get_logger().warn("RGB and depth sizes differ; ensure aligned topics")
            return

        # Reject external camera frames automatically
        rgb_frame_id = msg.header.frame_id or ''
        depth_frame_id = depth_header.frame_id or ''
        if 'external_camera' in rgb_frame_id or 'external_camera' in depth_frame_id:
            # Ignore external camera; use only TurtleBot camera frames
            return

        # Edge + contour detection (simple but robust)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 180)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        poses_cam = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cxp = int(x + w / 2)
            cyp = int(y + h / 2)

            # Median depth in small patch for robustness
            patch = depth_img[max(cyp-1, 0):cyp+2, max(cxp-1, 0):cxp+2]
            if patch.size == 0:
                continue
            depth = float(np.median(patch))
            if not np.isfinite(depth) or depth < self.min_depth or depth > self.max_depth:
                continue

            # Back-project to camera coordinates
            X = (cxp - self.cx) * depth / self.fx
            Y = (cyp - self.cy) * depth / self.fy
            Z = depth
            poses_cam.append((X, Y, Z))

        if not poses_cam:
            return

        # Transform to odom with robust TF handling
        pose_array_odom = PoseArray()
        pose_array_odom.header.frame_id = 'odom'
        pose_array_odom.header.stamp = msg.header.stamp

        # Determine source frame: prefer TurtleBot depth optical frame
        src_frame = self.tb_depth_optical
        # If the depth header has a valid TB frame, use it
        if depth_frame_id and 'camera_depth_optical_frame' in depth_frame_id:
            src_frame = depth_frame_id

        def try_lookup(time_obj):
            if not self.tf_buffer.can_transform('odom', src_frame, time_obj, timeout=rclpy.duration.Duration(seconds=0.5)):
                return None
            tf = self.tf_buffer.lookup_transform('odom', src_frame, time_obj)
            t = tf.transform.translation
            q = tf.transform.rotation
            R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            T = np.array([t.x, t.y, t.z], dtype=np.float32)
            return R, T

        # 1) Try exact image time; 2) fallback latest available TF (Time(0))
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        res = try_lookup(stamp)
        if res is None:
            self.get_logger().warn("TF at image stamp unavailable; falling back to latest TF")
            res = try_lookup(rclpy.time.Time())
            if res is None:
                self.get_logger().warn("TF transform to odom failed (no available transform)")
                return
        R, T = res

        # Project to ground plane if enabled (z=0) to match LIDAR abstractions
        for (xc, yc, zc) in poses_cam:
            p_cam = np.array([xc, yc, zc], dtype=np.float32)
            p_odom = R @ p_cam + T
            pose = Pose()
            pose.position.x = float(p_odom[0])
            pose.position.y = float(p_odom[1])
            pose.position.z = 0.0 if self.project_to_ground else float(p_odom[2])
            pose.orientation.w = 1.0
            pose_array_odom.poses.append(pose)

        self.pub_camera_odom.publish(pose_array_odom)
        self.get_logger().info(f"Published {len(pose_array_odom.poses)} camera obstacles in odom")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
