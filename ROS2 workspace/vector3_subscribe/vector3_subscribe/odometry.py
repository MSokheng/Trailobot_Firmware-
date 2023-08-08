#!/usr/bin/env python

import math
import rclpy
from math import sin, cos, pi
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quaternion = Quaternion()
    quaternion.w = cy * cp * cr + sy * sp * sr
    quaternion.x = cy * cp * sr - sy * sp * cr
    quaternion.y = sy * cp * sr + cy * sp * cr
    quaternion.z = sy * cp * cr - cy * sp * sr
    
    return quaternion

class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        self.subscription = self.create_subscription(
            Vector3,
            "/publish_twist",
            self.listener_callback,
            10
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def listener_callback(self, vector3_publish):
        linear_x = vector3_publish.x
        angular_z = vector3_publish.z

        delta_x = linear_x * cos(self.th) * 0.01
        delta_y = linear_x * sin(self.th) * 0.01
        delta_th = angular_z * 0.01

        self.x += delta_x 
        self.y += delta_y
        self.th += delta_th

        robot_orientation = quaternion_from_euler(0, 0, self.th)
        time = self.get_clock().now().to_msg()
        
        # transform
        transform = TransformStamped()
        transform.header.frame_id = "/odom"
        transform.child_frame_id = "/base_link"
        transform.header.stamp = time
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = robot_orientation
        
        # odometry
        odometry_message = Odometry()
        odometry_message.header.frame_id = "/odom"
        odometry_message.child_frame_id = "/base_link"
        odometry_message.header.stamp = time
        odometry_message.pose.pose.position.x = self.x
        odometry_message.pose.pose.position.y = self.y
        odometry_message.pose.pose.orientation = robot_orientation
        odometry_message.twist.twist.linear.x = linear_x
        odometry_message.twist.twist.angular.z = angular_z

        self.tf_broadcaster.sendTransform(transform)
        self.odom_publisher.publish(odometry_message)
        
        self.get_logger().info("position of robot:")
        self.get_logger().info("    position.x: %f" %(self.x))
        self.get_logger().info("    position.y: %f" %(self.y))
        self.get_logger().info("    head.angle: %f" %(self.th))

# first, we'll publish the transform over tf
def main(args=None):
    rclpy.init(args=args)
    robot_control_node = OdomPublisher()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
