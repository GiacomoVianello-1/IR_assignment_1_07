#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CorridorController(Node):
    def __init__(self):
        super().__init__('corridor_controller')

        # --- SUBSCRIBERS ---
        # Subscribe to the /corridor_active' topic published by the Corridor_Detector
        self.sub_corridor_flag = self.create_subscription(
            Bool, 
            '/corridor_active', 
            self.corridor_status_callback, 
            10
        )

        # --- PUBLISHERS ---
        # PPublish cmd_vel to control the robot directly
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- PARAMETRS ---
        self.forward_speed = 0.05   # m/s (constant velocity when in corridor)
        self.is_active = False      # internal state: are we currently controlling the robot?

        self.get_logger().info("Corridor_Controller started via /is_in_corridor flag")

    def corridor_status_callback(self, msg: Bool):

        in_corridor = msg.data

        if in_corridor:
            # --- IN CORRIDOR ---
            if not self.is_active:
                self.get_logger().info("Corridor detected: Taking control -> Forward Motion")
                self.is_active = True
            
            # Continue driving forward
            self.drive_forward()

        else:
            # --- SITUAZIONE: FUORI DAL CORRIDOIO / FINE CORRIDOIO ---
            if self.is_active:
                self.get_logger().info("Corridor ended: Stopping -> Releasing control to Nav2")
                self.is_active = False
                
                # 1. Robot Stop
                self.stop_robot()
                
            
            # Se non siamo attivi, ci assicuriamo che il robot stia fermo 
            # (o lasciamo che Nav2 pubblichi su cmd_vel, a seconda del tuo setup TwistMux)
            # Qui per sicurezza inviamo stop se il nodo è inteso come esclusivo.
            # self.stop_robot() 

    def drive_forward(self):
        
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0  # Vai dritto perfetto
        
        # Nota: In un corridoio reale, "dritto perfetto" potrebbe far sbattere il robot 
        # se entra storto. Un miglioramento futuro sarebbe leggere l'orientamento 
        # dei muri e correggere angular.z. Per ora, come richiesto, va solo avanti.
        self.pub_cmd_vel.publish(twist)

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
        # For safety, stop the robot on shutdown
        if rclpy.ok():
            # Temporary publisher to cmd_vel
            tmp_pub = node.create_publisher(Twist, 'cmd_vel', 10)
            stop_msg = Twist()
            tmp_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()