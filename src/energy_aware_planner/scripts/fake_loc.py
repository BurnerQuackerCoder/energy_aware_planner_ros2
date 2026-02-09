#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FakeLoc(Node):
    def __init__(self):
        super().__init__('fake_loc')
        self.br = StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback, 10)
        # Initialize at 0,0
        self.publish_tf(0.0, 0.0, 0.0)
        self.get_logger().info("Fake Localization Ready. Use '2D Pose Estimate' in RViz to move the robot.")

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"Moving robot to: {x}, {y}")
        self.publish_tf(x, y, 0.0)

    def publish_tf(self, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(FakeLoc())
    rclpy.shutdown()

if __name__ == '__main__':
    main()