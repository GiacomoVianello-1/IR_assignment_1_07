#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Bool

class CorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')

        # Subscriber: walls detected in odom frame
        self.sub_walls = self.create_subscription(
            PoseArray, 'table_detection/walls_odom', self.walls_callback, 10
        )

        # Publisher: velocity commands
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher: Nav2 enable/disable flag
        self.pub_nav2_enable = self.create_publisher(Bool, 'nav2_enable', 10)

        # Parameters
        self.forward_speed = 0.15   # m/s
        self.stability_cycles = 5   # number of cycles to confirm corridor state

        # State
        self.in_corridor = False
        self.counter_corridor = 0
        self.counter_exit = 0

        self.get_logger().info("Corridor_Controller started")

    def walls_callback(self, msg: PoseArray):
        num_walls = len(msg.poses)

        # Case: exactly 2 walls → corridor
        if num_walls == 2:
            self.counter_corridor += 1
            self.counter_exit = 0
            if self.counter_corridor >= self.stability_cycles and not self.in_corridor:
                self.in_corridor = True
                self.get_logger().info("Entering corridor: disabling Nav2, manual forward control")
                self.set_nav2_enabled(False)
            if self.in_corridor:
                self.drive_forward()

        # Case: more than 2 walls → corridor end
        elif num_walls > 2:
            self.counter_exit += 1
            self.counter_corridor = 0
            if self.counter_exit >= self.stability_cycles and self.in_corridor:
                self.in_corridor = False
                self.get_logger().info("Corridor ended: re-enabling Nav2")
                self.set_nav2_enabled(True)
                self.stop_robot()

        # Other cases (0 or 1 wall): stop robot, keep Nav2 enabled
        else:
            self.counter_corridor = 0
            self.counter_exit = 0
            if self.in_corridor:
                self.in_corridor = False
                self.get_logger().info("Corridor condition lost: re-enabling Nav2")
                self.set_nav2_enabled(True)
            self.stop_robot()

    def drive_forward(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def set_nav2_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.pub_nav2_enable.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
