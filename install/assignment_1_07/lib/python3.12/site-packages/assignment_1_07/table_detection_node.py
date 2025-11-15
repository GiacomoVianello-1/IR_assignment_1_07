#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, 'table_detection/poses', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Table Detection Node started, subscribed to /scan")

    def lidar_callback(self, msg: LaserScan):
        # esempio: prendi il punto più vicino come "gamba"
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + min_index * msg.angle_increment

        # coordinate nel frame del laser
        x = min_distance * cos(angle)
        y = min_distance * sin(angle)

        # crea PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = msg.header.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        try:
            # trasformazione nel frame /odom
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time())
            pose_odom = tf2_geometry_msgs.do_transform_pose(pose, transform)

            # pubblica
            self.publisher.publish(pose_odom)
            self.get_logger().info(f"Detected table leg at ({pose_odom.pose.position.x:.2f}, {pose_odom.pose.position.y:.2f}) wrt /odom")

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TableDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    from math import cos, sin
    main()
