#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Bool
import tf2_ros
import tf_transformations
import math

class SimpleCorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')

        # --- SUBSCRIBERS ---
        # Flag to activate/deactivate the controller
        self.sub_corridor_flag = self.create_subscription(
            Bool,
            '/corridor_active',
            self.corridor_status_callback,
            10
        )

        # Wall detections (PoseArray)
        self.sub_walls_odom = self.create_subscription(
            PoseArray,
            'table_detection/walls_odom',
            self.walls_callback,
            10
        )

        # --- PUBLISHER ---
        # Velocity commands
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- TF2 LISTENER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- PARAMETERS ---
        
        # Constant forward speed (m/s)
        self.declare_parameter('forward_speed', 0.25)
        
        # Proportional Gains (P)
        # KP ANGLE: Reacts to orientation error relative to walls.
        self.declare_parameter('kp_angle', 0.05)  # Increased slightly as D will dampen it                                      
        # KP CENTER: Reacts to lateral position error.
        self.declare_parameter('kp_centering', 0.4)
        
        # Derivative Gains (D)
        # KD ANGLE: Dampens angular oscillations.
        self.declare_parameter('kd_angle', 0.05)
        # KD CENTER: Dampens lateral oscillations.
        self.declare_parameter('kd_centering', 0.05)
        
        # Deadband (m): Ignore small lateral errors to prevent jitter.
        self.declare_parameter('lateral_deadband', 0.05)

        # Limits
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('control_rate', 20.0)

        # Load parameters
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.kp_angle = float(self.get_parameter('kp_angle').value)
        self.kp_centering = float(self.get_parameter('kp_centering').value)
        self.kd_angle = float(self.get_parameter('kd_angle').value)
        self.kd_centering = float(self.get_parameter('kd_centering').value)
        self.lateral_deadband = float(self.get_parameter('lateral_deadband').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # --- STATE ---
        self.is_active = False
        self.latest_walls = None
        self.latest_walls_frame = None
        
        # Previous error storage for Derivative calculation
        self.prev_phi = 0.0
        self.prev_mid_y = 0.0
        self.last_time = self.get_clock().now()

        # Timer setup
        period = 1.0 / max(1.0, self.control_rate)
        self.control_timer = self.create_timer(period, self.control_step)

        self.get_logger().info('CorridorController: PD Logic Initialized')

    def walls_callback(self, msg: PoseArray):
        """
        Callback to store the latest wall detections.
        """
        if msg is None or len(msg.poses) < 2:
            return
        self.latest_walls = msg
        # Fallback to 'map' if frame_id is empty
        self.latest_walls_frame = getattr(msg.header, 'frame_id', 'map')

    def corridor_status_callback(self, msg: Bool):
        """
        Callback to activate/deactivate the controller based on external flag.
        """
        flag = bool(msg.data)
        if flag != self.is_active:
            if flag:
                self.get_logger().info('Corridor detected: Moving forward')
                # Reset error history to avoid jumps when re-engaging
                self.prev_phi = 0.0
                self.prev_mid_y = 0.0
                self.last_time = self.get_clock().now()
            else:
                self.get_logger().info('Corridor lost: Stopping')
                self.stop_robot()
                self.destroy_node()
                rclpy.shutdown()
        self.is_active = flag

    def transform_point_to_base(self, px, py, source_frame):
        """
        Transforms a point (px, py) from source_frame to base_link.
        """
        try:
            # Lookup transform at current time
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform('base_link', source_frame, now)
            
            t = tf.transform.translation
            q = tf.transform.rotation
            # Convert quaternion to yaw
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            ca = math.cos(yaw)
            sa = math.sin(yaw)
            
            # Apply transform
            x_b = ca * px - sa * py + t.x
            y_b = sa * px + ca * py + t.y
            return x_b, y_b
        except Exception as e:
            # self.get_logger().warn(f'Transform error: {e}')
            return None, None

    def control_step(self):
        """
        Main control loop (PD Controller).
        """
        if not self.is_active:
            return

        # Safety Straight: if data is missing, move straight slowly
        if self.latest_walls is None or len(self.latest_walls.poses) < 2:
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            return

        # Extract wall points
        p1 = self.latest_walls.poses[0].position
        p2 = self.latest_walls.poses[1].position
        source_frame = self.latest_walls_frame if self.latest_walls_frame else 'odom'

        # Transform points to robot base frame
        p1_x, p1_y = self.transform_point_to_base(p1.x, p1.y, source_frame)
        p2_x, p2_y = self.transform_point_to_base(p2.x, p2.y, source_frame)

        if p1_x is None or p2_x is None:
            return

        # --- PD LOGIC ---

        # 1. Calculate Time Step (dt)
        current_time = self.get_clock().now()
        # Convert duration to seconds (float)
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Avoid division by zero or huge jumps
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05 # Fallback to expected period

        # 2. Calculate Errors (Proportional Term)
        
        # Lateral Error (Distance from center)
        mid_y = 0.5 * (p1_y + p2_y)
        
        # Heading Error (Angle relative to corridor)
        dx = p2_x - p1_x
        dy = p2_y - p1_y
        phi = math.atan2(dy, dx)
        
        # Normalize angle to [-pi/2, pi/2] for robust tracking
        if phi > math.pi / 2.0:
            phi -= math.pi
        elif phi < -math.pi / 2.0:
            phi += math.pi

        # 3. Apply Deadband to Lateral Error
        # If error is small, treat P-error as 0 (but keep calculating D-term for stability)
        effective_mid_y = mid_y
        if abs(mid_y) < self.lateral_deadband:
            effective_mid_y = 0.0

        # 4. Calculate Derivatives (D Term)
        # Rate of change = (current - previous) / dt
        d_phi = (phi - self.prev_phi) / dt
        d_mid_y = (mid_y - self.prev_mid_y) / dt

        # 5. Compute Control Command
        # Command = P_term + D_term
        
        # Angle Control (Keep parallel)
        angle_correction = (-self.kp_angle * phi) + (-self.kd_angle * d_phi)
        
        # Centering Control (Keep centered)
        center_correction = (-self.kp_centering * effective_mid_y) + (-self.kd_centering * d_mid_y)
        
        # Total Angular Velocity
        angular_z = angle_correction + center_correction

        # 6. Store values for next iteration
        self.prev_phi = phi
        self.prev_mid_y = mid_y
        self.last_time = current_time

        # Clamp max angular speed for safety
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        # Publish Command
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        self.pub_cmd_vel.publish(twist)

        # Debug logging
        # self.get_logger().debug(
        #     f'PD Logic | P_phi: {phi:.3f}, D_phi: {d_phi:.3f} | '
        #     f'P_y: {effective_mid_y:.3f}, D_y: {d_mid_y:.3f} | '
        #     f'CMD: {angular_z:.3f}'
        # )

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocity.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCorridorController()
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