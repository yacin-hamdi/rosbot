#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class PublishTwistVel(Node):
    def __init__(self):
        super().__init__("publish_twist_vel")
        self.vel_sub_ = self.create_subscription(Twist, 
                                                 "/cmd_vel", 
                                                 self.velCallback, 
                                                 10)
        
        self.vel_pub_ = self.create_publisher(TwistStamped, 
                                              "/rosbot_controller/cmd_vel", 
                                              10)
        

    def velCallback(self, msg: Twist):
        cmd_vel_stamped = TwistStamped()
        cmd_vel_stamped.twist = msg
        cmd_vel_stamped.header.stamp = self.get_clock().now().to_msg()

        self.vel_pub_.publish(cmd_vel_stamped)


def main():
    rclpy.init()
    node = PublishTwistVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()