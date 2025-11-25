#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped
from std_msgs.msg import Bool

import tf2_ros
import tf_transformations
from math import cos, sin, atan2, pi
import numpy as np
import random
import time

# RANSAC and corridor detection parameters
DEFAULT_DIST_THRESHOLD = 0.02
DEFAULT_MIN_INLIERS = 50
DEFAULT_MAX_ITER = 500
DEFAULT_SIDE_ANGLE_RANGE = (0.2, 1.4)   # rad
DEFAULT_MIN_RANGE = 0.05
DEFAULT_MAX_RANGE = 8.5
DEFAULT_CONFIRM_FRAMES = 5              # number of consecutive frames to confirm corridor
DEFAULT_LOST_FRAMES = 9                 # number of consecutive frames to declare lost

class CorridorDetector(Node):
    def __init__(self):
        super().__init__('corridor_detector')

        # Params 
        self.declare_parameter('dist_threshold', DEFAULT_DIST_THRESHOLD)
        self.declare_parameter('min_inliers', DEFAULT_MIN_INLIERS)
        self.declare_parameter('max_iter', DEFAULT_MAX_ITER)
        self.declare_parameter('min_range', DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', DEFAULT_MAX_RANGE)
        self.declare_parameter('side_angle_min', DEFAULT_SIDE_ANGLE_RANGE[0])
        self.declare_parameter('side_angle_max', DEFAULT_SIDE_ANGLE_RANGE[1])
        self.declare_parameter('publish_odom_frame', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('confirm_frames', DEFAULT_CONFIRM_FRAMES)
        self.declare_parameter('lost_frames', DEFAULT_LOST_FRAMES)

        # Read param values
        self.dist_threshold = self.get_parameter('dist_threshold').value
        self.min_inliers = self.get_parameter('min_inliers').value
        self.max_iter = self.get_parameter('max_iter').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.side_angle_min = self.get_parameter('side_angle_min').value
        self.side_angle_max = self.get_parameter('side_angle_max').value
        self.publish_odom_frame = self.get_parameter('publish_odom_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.lost_frames = int(self.get_parameter('lost_frames').value)

        # Subscription
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )

        # Publishers
        self.pub_is_corridor = self.create_publisher(Bool, '/corridor_active', 10)
        self.pub_walls_src = self.create_publisher(PoseArray, 'table_detection/walls', 10)
        self.pub_walls_odom = self.create_publisher(PoseArray, 'table_detection/walls_odom', 10)

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # RNG
        random.seed()

        # Internal state for temporal caching
        self._consecutive_positive = 0
        self._consecutive_negative = 0
        self._published_state = False  # last published corridor_active
        self.get_logger().info('CorridorDetector initialized (temporal caching enabled)')

    # Convert ranges+angles to XY points in laser frame, filtering by angle and range
    def ranges_to_xy(self, ranges, angle_min, angle_increment):
        xs = []
        ys = []
        for i, r in enumerate(ranges):
            if not np.isfinite(r):
                continue
            if r < self.min_range or r > self.max_range:
                continue
            angle = angle_min + i * angle_increment
            # Keep only side angles (exclude frontal noisy region)
            a = angle
            # normalize to [-pi,pi]
            a_n = (a + pi) % (2*pi) - pi
            if (abs(a_n) < self.side_angle_min) or (abs(a_n) > self.side_angle_max):
                continue
            x = r * cos(a)
            y = r * sin(a)
            xs.append(x)
            ys.append(y)
        if len(xs) == 0:
            return np.empty((0,2))
        return np.vstack((xs, ys)).T

    # Distance from points to line ax+by+c=0
    def point_line_distance(self, pts, line):
        a, b, c = line
        num = np.abs(a * pts[:,0] + b * pts[:,1] + c)
        den = np.sqrt(a*a + b*b)
        return num / den

    # Construct normalized line from two points
    def line_from_two_points(self, p1, p2):
        x1,y1 = p1; x2,y2 = p2
        if np.allclose([x1,y1], [x2,y2]):
            return None
        a = y1 - y2
        b = x2 - x1
        c = x1*y2 - x2*y1
        norm = np.hypot(a, b)
        if norm == 0:
            return None
        return (a / norm, b / norm, c / norm)

    # Simple RANSAC for line detection
    def ransac_line(self, pts):
        n = pts.shape[0]
        if n < 2:
            return None, np.array([], dtype=int)
        best_line = None
        best_inliers = np.array([], dtype=int)
        for _ in range(self.max_iter):
            i1, i2 = random.sample(range(n), 2)
            line = self.line_from_two_points(pts[i1], pts[i2])
            if line is None:
                continue
            dists = self.point_line_distance(pts, line)
            inliers_idx = np.where(dists <= self.dist_threshold)[0]
            if inliers_idx.size > best_inliers.size:
                best_inliers = inliers_idx
                best_line = line
                if best_inliers.size >= n * 0.6:
                    break
        return best_line, best_inliers

    # Convert line to Pose (closest point to origin + orientation along line)
    def line_to_pose(self, line):
        a, b, c = line
        den = a*a + b*b
        px = -a * c / den
        py = -b * c / den
        dx = b
        dy = -a
        yaw = atan2(dy, dx)
        pose = Pose()
        pose.position.x = float(px)
        pose.position.y = float(py)
        pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    # Remove inliers from point set
    def remove_inliers(self, pts, inlier_idx):
        mask = np.ones(pts.shape[0], dtype=bool)
        mask[inlier_idx] = False
        return pts[mask]

    # Transform PoseStamped to target frame using TF2 (best-effort)
    def transform_pose(self, pose_stamped, target_frame):
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            mat_trans = tf_transformations.translation_matrix((t.x, t.y, t.z))
            mat_rot = tf_transformations.quaternion_matrix((r.x, r.y, r.z, r.w))
            M = np.dot(mat_trans, mat_rot)
            p = np.array([pose_stamped.pose.position.x,
                          pose_stamped.pose.position.y,
                          pose_stamped.pose.position.z, 1.0])
            p2 = M.dot(p)
            q_orig = (pose_stamped.pose.orientation.x,
                      pose_stamped.pose.orientation.y,
                      pose_stamped.pose.orientation.z,
                      pose_stamped.pose.orientation.w)
            q_new = tf_transformations.quaternion_multiply((r.x, r.y, r.z, r.w), q_orig)
            new_pose = Pose()
            new_pose.position.x = float(p2[0])
            new_pose.position.y = float(p2[1])
            new_pose.position.z = float(p2[2])
            new_pose.orientation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])
            return new_pose
        except Exception:
            return None

    # Main LIDAR callback
    def lidar_callback(self, msg: LaserScan):
        t0 = time.time()
        pts = self.ranges_to_xy(msg.ranges, msg.angle_min, msg.angle_increment)

        # Prepare PoseArray headers
        walls_src = PoseArray()
        walls_src.header.stamp = msg.header.stamp
        walls_src.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.laser_frame

        walls_odom = PoseArray()
        walls_odom.header.stamp = msg.header.stamp
        walls_odom.header.frame_id = self.odom_frame

        corridor_detected_now = False

        # Quick exit if not enough points
        if pts.shape[0] < 10:
            corridor_detected_now = False
            self.get_logger().debug('Not enough points for detection')
            self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)
            return

        # First line
        line1, inliers1 = self.ransac_line(pts)
        if line1 is None or inliers1.size < self.min_inliers:
            corridor_detected_now = False
            self.get_logger().debug('No dominant first line found')
            self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)
            return

        # Remove first inliers and find second
        rem_pts = self.remove_inliers(pts, inliers1)
        line2, inliers2 = self.ransac_line(rem_pts)

        if line2 is None or inliers2.size < self.min_inliers:
            # Fall-back: cluster left/right by sign of y and try ransac separately
            left_pts = pts[pts[:,1] > 0]
            right_pts = pts[pts[:,1] < 0]
            alt_l = None; alt_r = None
            if left_pts.shape[0] >= 2:
                l, li = self.ransac_line(left_pts)
                if l is not None and li.size >= self.min_inliers:
                    alt_l = l
            if right_pts.shape[0] >= 2:
                r, ri = self.ransac_line(right_pts)
                if r is not None and ri.size >= self.min_inliers:
                    alt_r = r
            if alt_l is not None and alt_r is not None:
                line1 = alt_l
                line2 = alt_r
            else:
                corridor_detected_now = False
                self.get_logger().debug('Second line not found reliably')
                self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)
                return

        # Ensure roughly parallel
        def line_angle(line):
            a,b,c = line
            dx, dy = b, -a
            return atan2(dy, dx)
        def normalize_half(a):
            a = (a + pi) % pi - pi/2
            return a

        ang1 = normalize_half(line_angle(line1))
        ang2 = normalize_half(line_angle(line2))
        ang_diff = abs(ang1 - ang2)
        if ang_diff > 0.3:
            # try flipping sign of line2
            line2_alt = (-line2[0], -line2[1], -line2[2])
            ang2_alt = normalize_half(line_angle(line2_alt))
            if abs(ang1 - ang2_alt) < ang_diff:
                line2 = line2_alt
                ang_diff = abs(ang1 - ang2_alt)
        if ang_diff > 0.5:
            corridor_detected_now = False
            self.get_logger().debug(f'Lines not parallel: diff={ang_diff:.2f}')
            self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)
            return

        # Ensure lines are on opposite sides of robot (sign of c)
        side1 = np.sign(line1[2])
        side2 = np.sign(line2[2])
        if side1 == 0:
            side1 = 1
        if side2 == 0:
            side2 = -1
        if side1 == side2:
            # try left/right clustering fallback
            left_pts = pts[pts[:,1] > 0]
            right_pts = pts[pts[:,1] < 0]
            alt_l = None; alt_r = None
            if left_pts.shape[0] >= 2:
                l, li = self.ransac_line(left_pts)
                if l is not None and li.size >= self.min_inliers:
                    alt_l = l
            if right_pts.shape[0] >= 2:
                r, ri = self.ransac_line(right_pts)
                if r is not None and ri.size >= self.min_inliers:
                    alt_r = r
            if alt_l is not None and alt_r is not None:
                line1 = alt_l
                line2 = alt_r
            else:
                corridor_detected_now = False
                self.get_logger().debug('Lines on same side and no alternate found')
                self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)
                return

        # If we reach here, we have two valid lines: corridor detected in this frame
        corridor_detected_now = True

        # Convert to PoseArray (source frame)
        pose1 = self.line_to_pose(line1)
        pose2 = self.line_to_pose(line2)
        # Order left/right by y coordinate (positive y = left in robot frame)
        if pose1.position.y < pose2.position.y:
            left_pose = pose2
            right_pose = pose1
        else:
            left_pose = pose1
            right_pose = pose2

        walls_src.poses = [left_pose, right_pose]
        # Publish walls in source frame immediately (useful for controller)
        self.pub_walls_src.publish(walls_src)

        # Try to publish walls in odom frame (best-effort)
        if self.publish_odom_frame:
            walls_odom.poses = []
            for p in walls_src.poses:
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = walls_src.header.frame_id
                ps.pose = p
                transformed = self.transform_pose(ps, self.odom_frame)
                if transformed is None:
                    walls_odom.poses = []
                    break
                walls_odom.poses.append(transformed)
            if len(walls_odom.poses) == 2:
                self.pub_walls_odom.publish(walls_odom)
            else:
                # publish empty/partial to keep topic alive (choice made intentionally)
                self.pub_walls_odom.publish(walls_odom)

        # Update temporal confirmation state and publish corridor flag only after confirmation
        self._update_temporal_state(corridor_detected_now, walls_src, walls_odom, msg)

        t_total = time.time() - t0
        self.get_logger().debug(f'Frame processed in {t_total:.03f}s, corridor_now={corridor_detected_now}')

    # Temporal caching: update counters and publish Bool only when confirmed or lost by thresholds
    def _update_temporal_state(self, detected_now, walls_src, walls_odom, scan_msg):
        if detected_now:
            self._consecutive_positive += 1
            self._consecutive_negative = 0
        else:
            self._consecutive_negative += 1
            self._consecutive_positive = 0

        # If not yet published True and positive count reached threshold -> publish True
        if (not self._published_state) and (self._consecutive_positive >= self.confirm_frames):
            b = Bool()
            b.data = True
            self.pub_is_corridor.publish(b)
            self._published_state = True
            self.get_logger().info(f'Corridor confirmed after {self._consecutive_positive} frames -> published True')
            # Optionally re-publish walls at confirmation time (already published source walls above)
            self.pub_walls_src.publish(walls_src)
            if len(walls_odom.poses) == 2:
                self.pub_walls_odom.publish(walls_odom)
            return

        # If currently published True but lost frames reached threshold -> publish False
        if self._published_state and (self._consecutive_negative >= self.lost_frames):
            b = Bool()
            b.data = False
            self.pub_is_corridor.publish(b)
            self._published_state = False
            self.get_logger().info(f'Corridor lost after {self._consecutive_negative} frames -> published False')
            # optionally publish empty walls
            empty_src = PoseArray()
            empty_src.header.stamp = scan_msg.header.stamp
            empty_src.header.frame_id = scan_msg.header.frame_id if scan_msg.header.frame_id else self.laser_frame
            self.pub_walls_src.publish(empty_src)
            empty_odom = PoseArray()
            empty_odom.header.stamp = scan_msg.header.stamp
            empty_odom.header.frame_id = self.odom_frame
            self.pub_walls_odom.publish(empty_odom)
            return

        # If neither threshold crossed, do not change published corridor flag.
        # However, for initial state (never published anything) we keep publishing False periodically to ensure a value exists.
        # Publish a False once at startup if never published True and counters are small
        if (not self._published_state) and (self._consecutive_positive == 0 and self._consecutive_negative == 1):
            # publish explicit False to initialize topic consumers
            b = Bool(); b.data = False
            self.pub_is_corridor.publish(b)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
