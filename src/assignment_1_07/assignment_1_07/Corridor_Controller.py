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
        self.declare_parameter('forward_speed', 0.18)      # m/s
        self.declare_parameter('kp_lateral', 0.6)         # P gain for lateral error (mid_y) -> angular z (RIDOTTO per meno aggressività)
        self.declare_parameter('ki_lateral', 0.15)        # I gain for lateral error integral -> angular z # NUOVO PARAMETRO
        self.declare_parameter('kp_angle', 1.0)           # P gain for angular error (phi) -> angular z (RIDOTTO per meno aggressività)
        self.declare_parameter('max_angular', 0.7)        # rad/s (RIDOTTO per meno oscillazioni)
        self.declare_parameter('control_rate', 10.0)      # Hz
        self.declare_parameter('max_integral', 0.5)       # Max value for the integral term (anti-windup) # NUOVO PARAMETRO

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.kp_lateral = float(self.get_parameter('kp_lateral').value)
        self.ki_lateral = float(self.get_parameter('ki_lateral').value) # Nuovo Ki
        self.kp_angle = float(self.get_parameter('kp_angle').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_integral = float(self.get_parameter('max_integral').value) # Anti-windup limit

        # --- STATE ---
        self.is_active = False              # True when corridor detected
        self.latest_walls = None            # last received PoseArray
        self.latest_walls_frame = None      # frame_id of latest_walls.header
        self.integral_error_y = 0.0         # Accumulator for lateral error (y_mid) # STATO INTEGRALE
        
        # Periodic control timer setup
        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.control_timer = self.create_timer(self.control_period, self.control_step)

        self.get_logger().info('CorridorController initialized with P-I (Lateral) + P (Angular)')

    # store latest walls (PoseArray)
    def walls_callback(self, msg: PoseArray):
        if msg is None or len(msg.poses) < 2:
            self.get_logger().debug('walls_callback: <2 poses, ignoring')
            return
        self.latest_walls = msg
        self.latest_walls_frame = getattr(msg.header, 'frame_id', 'odom')

    # corridor active flag
    def corridor_status_callback(self, msg: Bool):
        flag = bool(msg.data)
        if flag and not self.is_active:
            self.get_logger().info('Corridor detected: controller engaged')
        if not flag and self.is_active:
            self.get_logger().info('Corridor lost: controller disengaged and stopping')
            # Reset integrale quando il corridoio non è più attivo
            self.integral_error_y = 0.0 
            self.stop_robot()
        self.is_active = flag

    # transform a 2D point (px,py) in source_frame into base_link frame (OMESSO PER BREVITA')
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
        except Exception as e:
            self.get_logger().debug(f'transform_point_to_base: transform error: {e}')
            return None, None

    # main periodic control step
    def control_step(self):
        if not self.is_active:
            # Assicurati che l'integratore sia resettato se inattivo
            self.integral_error_y = 0.0
            return

        if self.latest_walls is None or len(self.latest_walls.poses) < 2:
            self.get_logger().debug('control_step: no walls available, publishing forward only')
            twist = Twist()
            twist.linear.x = float(self.forward_speed)
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        p1 = self.latest_walls.poses[0].position
        p2 = self.latest_walls.poses[1].position
        source_frame = self.latest_walls_frame if self.latest_walls_frame else 'odom'

        p1_x, p1_y = self.transform_point_to_base(p1.x, p1.y, source_frame)
        p2_x, p2_y = self.transform_point_to_base(p2.x, p2.y, source_frame)

        if p1_x is None or p2_x is None:
            self.get_logger().debug('control_step: TF unavailable, publishing forward only')
            twist = Twist()
            twist.linear.x = float(self.forward_speed)
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        # --- LOGICA DI FEEDBACK P-I (Laterale) + P (Angolare) ---

        # 1. Errore Laterale: y coordinata del punto medio (mid_y)
        mid_y = 0.5 * (p1_y + p2_y)

        # 2. Errore Angolare: Angolo phi
        dx = p2_x - p1_x
        dy = p2_y - p1_y
        phi = math.atan2(dy, dx)
        
        # Correzione del quadrante per errore angolare minimo
        if phi > math.pi / 2.0:
            phi -= math.pi
        elif phi < -math.pi / 2.0:
            phi += math.pi

        # 3. Calcolo del Termine Integrale (per l'Errore Laterale)
        
        # Accumulo dell'errore (integrazione rettangolare semplice: errore * tempo)
        self.integral_error_y += mid_y * self.control_period
        
        # Anti-windup (saturazione dell'integratore)
        self.integral_error_y = max(min(self.integral_error_y, self.max_integral), -self.max_integral)
        
        # Calcolo dei termini del controllo
        angular_z_lateral_p = - self.kp_lateral * mid_y
        angular_z_lateral_i = - self.ki_lateral * self.integral_error_y
        angular_z_angle_p = - self.kp_angle * phi
        
        # Comando angolare totale
        angular_z = angular_z_lateral_p + angular_z_lateral_i + angular_z_angle_p
        
        # Saturazione di angular_z
        angular_z = max(min(angular_z, self.max_angular), -self.max_angular)

        # --- FINE LOGICA DI FEEDBACK ---

        # build and publish Twist
        twist = Twist()
        twist.linear.x = float(self.forward_speed)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)

        # debug
        self.get_logger().debug(
            f'control_step: mid_y={mid_y:.3f}, Integral={self.integral_error_y:.3f}, phi={phi:.3f}, '
            f'P_lat={angular_z_lateral_p:.3f}, I_lat={angular_z_lateral_i:.3f}, P_ang={angular_z_angle_p:.3f}, '
            f'cmd_ang={twist.angular.z:.3f}'
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