#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Bool
import tf2_ros
import tf_transformations
import math

class SimpleCorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')

        # --- SUBSCRIBERS ---
        self.sub_corridor_flag = self.create_subscription(
            Bool,
            '/corridor_active',
            self.corridor_status_callback,
            10
        )

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
        # Constant forward speed 
        self.declare_parameter('forward_speed', 0.25)
        
        # KP ANGLE: High value. Keeps the robot strictly parallel to walls.
        self.declare_parameter('kp_angle', 0)                                       # CAMBIATOOOOOOOOOOOOOOOOO
        
        # KP CENTER: Low value. Only gently pushes robot back if wheel drift occurs.
        self.declare_parameter('kp_centering', 0.1)
        
        # DEADBAND: Error threshold [m].
        # If lateral error is less than this (e.g., 5cm), ignore it.
        # This stops the robot from shimmying/hunting for perfect zero.
        self.declare_parameter('lateral_deadband', 0.5)

        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('control_rate', 20.0)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.kp_angle = float(self.get_parameter('kp_angle').value)
        self.kp_centering = float(self.get_parameter('kp_centering').value)
        self.lateral_deadband = float(self.get_parameter('lateral_deadband').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # --- STATE ---
        self.is_active = False
        self.latest_walls = None
        self.latest_walls_frame = None

        # Timer
        period = 1.0 / max(1.0, self.control_rate)
        self.control_timer = self.create_timer(period, self.control_step)

        #self.get_logger().info('SimpleCorridorController: Heading-Priority Logic Initialized')

    def walls_callback(self, msg: PoseArray):
        if msg is None or len(msg.poses) < 2:
            return
        self.latest_walls = msg
        self.latest_walls_frame = getattr(msg.header, 'frame_id', 'odom')

    def corridor_status_callback(self, msg: Bool):
        flag = bool(msg.data)
        if flag != self.is_active:
            if flag:
                self.get_logger().info('Corridor detected: Moving forward')
            else:
                self.get_logger().info('Corridor lost: Stopping')
                self.stop_robot()
        self.is_active = flag

    def transform_point_to_base(self, px, py, source_frame):
        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform('base_link', source_frame, now)
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            ca = math.cos(yaw)
            sa = math.sin(yaw)
            x_b = ca * px - sa * py + t.x
            y_b = sa * px + ca * py + t.y
            return x_b, y_b
        except Exception:
            return None, None

    def control_step(self):
        if not self.is_active:
            return

        # If no walls detected, perform "Safety Straight": move slowly forward without turning
        # assuming we are still in the corridor but lost detection momentarily.
        if self.latest_walls is None or len(self.latest_walls.poses) < 2:
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            return

        p1 = self.latest_walls.poses[0].position
        p2 = self.latest_walls.poses[1].position
        source_frame = self.latest_walls_frame if self.latest_walls_frame else 'odom'

        p1_x, p1_y = self.transform_point_to_base(p1.x, p1.y, source_frame)
        p2_x, p2_y = self.transform_point_to_base(p2.x, p2.y, source_frame)

        if p1_x is None or p2_x is None:
            return

        # --- SIMPLIFIED ROBUST LOGIC ---

        # 1. Calculate Lateral Error (Distance from center)
        mid_y = 0.5 * (p1_y + p2_y)

        # 2. Calculate Heading Error (Angle relative to corridor)
        dx = p2_x - p1_x
        dy = p2_y - p1_y
        phi = math.atan2(dy, dx)
        
        # Normalize angle to [-pi/2, pi/2]
        if phi > math.pi / 2.0:
            phi -= math.pi
        elif phi < -math.pi / 2.0:
            phi += math.pi

        # 3. Apply Deadband to Lateral Error
        # If we are within +/- 5cm (or set value) of the center, consider error as 0.
        # This prevents the robot from trying to fix microscopic errors.
        effective_mid_y = mid_y
        if abs(mid_y) < self.lateral_deadband:
            effective_mid_y = 0.0

        # 4. Compute Control Command
        # Strong correction on Angle (Keep straight) + Weak correction on Position (Fix drift)
        
        term_angle = -self.kp_angle * phi
        term_center = -self.kp_centering * effective_mid_y
        
        angular_z = term_angle + term_center

        # Clamp max speed
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        # Publish
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = angular_z
        self.pub_cmd_vel.publish(twist)

        # self.get_logger().debug( f'Logic: phi={phi:.3f}, mid_y={mid_y:.3f}, eff_y={effective_mid_y:.3f} -> cmd={angular_z:.3f}'    )

    def stop_robot(self):
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