#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray, PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import tf_transformations
import numpy as np
import math

class CorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')

        # --- SUBSCRIBERS ---
        self.sub_corridor_flag = self.create_subscription(
            Bool,
            '/corridor_active',
            self.corridor_status_callback,
            10
        )

        # PoseArray coming from detector, expected in some frame (usually 'odom')
        self.sub_walls_odom = self.create_subscription(
            PoseArray,
            'table_detection/walls_odom',
            self.walls_callback,
            10
        )

        # --- PUBLISHER ---
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- TF2 LISTENER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- PARAMETERS ---
        self.declare_parameter('forward_speed', 0.2)        # m/s
        self.declare_parameter('kp_lateral', 0.3)           # P gain for lateral error (mid_y) -> angular z
        self.declare_parameter('kp_angle', 0.8)             # P gain for angular error (phi) -> angular z 
        self.declare_parameter('max_angular', 0.3)          # rad/s
        self.declare_parameter('control_rate', 10)          # Hz

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.kp_lateral = float(self.get_parameter('kp_lateral').value) 
        self.kp_angle = float(self.get_parameter('kp_angle').value)     
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # --- STATE ---
        self.is_active = False             # True when corridor detected
        self.latest_walls = None           # last received PoseArray
        self.latest_walls_frame = None     # frame_id of latest_walls.header

        # Periodic controller timer (keeps publishing while active)
        period = 1.0 / max(1.0, self.control_rate)
        self.control_timer = self.create_timer(period, self.control_step)

        self.get_logger().info('CorridorController initialized')

    # Stores the latest walls data (PoseArray)
    def walls_callback(self, msg: PoseArray):
        if msg is None:
            return
        if len(msg.poses) < 2:
            # Ignore incomplete messages but keep previous data if it exists
            self.get_logger().debug('walls_callback: <2 poses, ignoring')
            return
        self.latest_walls = msg
        # Remember frame id so the transform uses the correct source frame
        self.latest_walls_frame = getattr(msg.header, 'frame_id', 'odom')

    # Corridor active flag status update
    def corridor_status_callback(self, msg: Bool):
        flag = bool(msg.data)
        if flag and not self.is_active:
            self.get_logger().info('Corridor detected: controller engaged')
        if not flag and self.is_active:
            self.get_logger().info('Corridor lost: controller disengaged and stopping')
            # Immediate stop when corridor following ends
            self.stop_robot()
        self.is_active = flag

    # Transforms a 2D point (px,py) from source_frame to base_link frame
    def transform_point_to_base(self, px, py, source_frame):
        try:
            # Request transform from source_frame -> base_link
            # lookup_transform(target_frame, source_frame, time)
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform('base_link', source_frame, now)
            t = tf.transform.translation
            q = tf.transform.rotation
            # Get yaw angle (rotation from source_frame to base_link)
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            # p_base = R * p_source + t (R rotates from source -> base)
            ca = math.cos(yaw)
            sa = math.sin(yaw)
            x_b = ca * px - sa * py + t.x
            y_b = sa * px + ca * py + t.y
            return x_b, y_b
        except Exception as e:
            # Transform unavailable or stale
            self.get_logger().debug(f'transform_point_to_base: transform error: {e}')
            return None, None

    # Main periodic control step
    def control_step(self):
        # Not active -> do nothing (but ensure stopped by stop_robot on deactivation)
        if not self.is_active:
            return

        # If wall data is unavailable -> don't stop; keep moving forward (simple strategy)
        if self.latest_walls is None or len(self.latest_walls.poses) < 2:
            self.get_logger().debug('control_step: no walls available, publishing forward only')
            twist = Twist()
            twist.linear.x = float(self.forward_speed)
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        # Extract the two wall poses (in the odometry frame). Order is NOT guaranteed.
        p1 = self.latest_walls.poses[0].position
        p2 = self.latest_walls.poses[1].position
        source_frame = self.latest_walls_frame if self.latest_walls_frame else 'odom'

        # Transform both points into the base_link frame
        p1_x, p1_y = self.transform_point_to_base(p1.x, p1.y, source_frame)
        p2_x, p2_y = self.transform_point_to_base(p2.x, p2.y, source_frame)

        # If the transformation failed -> fallback: publish forward only
        if p1_x is None or p2_x is None:
            self.get_logger().debug('control_step: TF unavailable, publishing forward only')
            twist = Twist()
            twist.linear.x = float(self.forward_speed)
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        # --- ENHANCED FEEDBACK AND CONTROL LOGIC ---

        # 1. Lateral Error: y-coordinate of the midpoint (mid_y)
        # mid_x is needed for angle calculation if atan2 were calculated differently, but mid_y is the lateral error
        mid_x = 0.5 * (p1_x + p2_x)
        mid_y = 0.5 * (p1_y + p2_y)

        # 2. Angular Error: Angle of the line connecting p1 and p2 relative to the robot's X-axis
        # dx and dy are the components of the vector from p1 to p2 in the base_link frame
        dx = p2_x - p1_x
        dy = p2_y - p1_y
        
        # phi is the corridor angle relative to the robot's X-axis (direction of travel)
        # atan2 returns the angle in [-pi, pi]
        phi = math.atan2(dy, dx) 

        # Quadrant Correction: if the robot is going in the opposite direction of detection
        # Ensure the angular error is always the smallest possible value for alignment
        # E.g., an angle of -170 degrees is nearly +10 degrees relative to alignment.
        if phi > math.pi / 2.0:
            phi -= math.pi
        elif phi < -math.pi / 2.0:
            phi += math.pi
        # After this correction, phi represents the angular error (should be 0 for perfect alignment)

        # 3. Calculate Angular Command (P + P Control)
        
        # P term on lateral error (centering)
        # Rotation is opposite to the error: if mid_y > 0 (robot too far left), angular_z < 0 (turn right)
        angular_z_lateral = - self.kp_lateral * mid_y
        
        # P term on angular error (parallel alignment)
        # Rotation is opposite to the error: if phi > 0 (corridor oriented left), angular_z < 0 (turn right)
        angular_z_angle = - self.kp_angle * phi
        
        # Total angular command
        angular_z = angular_z_lateral + angular_z_angle
        
        # Saturate angular_z to prevent excessive rotation
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        # --- END OF ENHANCED FEEDBACK AND CONTROL LOGIC ---

        # Build and publish Twist message
        twist = Twist()
        twist.linear.x = float(self.forward_speed)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)

        # Debug log: current state and commands
        self.get_logger().debug(
            f'control_step: p1=({p1_x:.3f},{p1_y:.3f}) p2=({p2_x:.3f},{p2_y:.3f}) mid=({mid_x:.3f},{mid_y:.3f}) '
            f'phi={phi:.3f} ang_lat={angular_z_lateral:.3f} ang_ang={angular_z_angle:.3f} '
            f'cmd_lin={twist.linear.x:.3f} cmd_ang={twist.angular.z:.3f}'
        )

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()