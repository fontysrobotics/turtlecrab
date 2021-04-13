import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
from tf2_ros import TransformBroadcaster, TransformStamped
from builtin_interfaces.msg import Time

import os, mmap
from struct import Struct

class HoloDriver(Node):
    def __init__(self):
        super().__init__('holo_driver')
        self.fd_velocity = os.open("/run/user/1000/holo_velocity", os.O_RDWR|os.O_NONBLOCK)
        self.velocity_struct = Struct('fff')
        self.velocity_block = mmap.mmap(self.fd_velocity, self.velocity_struct.size)
        self.create_subscription(Twist, '/cmd_vel', self.on_velocity, 10)
        
        self.create_timer(0.01, self.on_odom, None, None)
        self.fd_odom = os.open("/run/user/1000/holo_odom", os.O_RDWR|os.O_NONBLOCK)
        self.odom_struct = Struct('ffffff')
        self.odom_block = mmap.mmap(self.fd_odom, self.odom_struct.size)
        self.create_timer(0.01, self.on_odom, None, None)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_odom_obj = Odometry()
        self.tf_odom = TransformBroadcaster(self)
        self.tf_odom_obj = TransformStamped()

    def on_velocity(self, data):
        pack = self.velocity_struct.pack(data.linear.x, data.linear.y, data.angular.z)
        self.velocity_block[:] = pack

    def on_odom(self):
        odom_val = self.odom_struct.unpack(self.odom_block[:])
        quat = euler2quat(0.0, 0.0, odom_val[2])
        stamp = self.get_clock().now().seconds_nanoseconds()
        
        self.pub_odom_obj.header.frame_id = "odom"
        self.pub_odom_obj.pose.pose.position.x = odom_val[0]
        self.pub_odom_obj.pose.pose.position.y = odom_val[1]
        self.pub_odom_obj.pose.pose.orientation.x = quat[1]
        self.pub_odom_obj.pose.pose.orientation.y = quat[2]
        self.pub_odom_obj.pose.pose.orientation.z = quat[3]
        self.pub_odom_obj.pose.pose.orientation.w = quat[0]
        self.pub_odom_obj.twist.twist.linear.x = odom_val[3]
        self.pub_odom_obj.twist.twist.linear.x = odom_val[4]
        self.pub_odom_obj.twist.twist.linear.x = odom_val[5]
        self.pub_odom.publish(self.pub_odom_obj)

        self.tf_odom_obj.header.frame_id = "map"
        self.tf_odom_obj.header.stamp = Time(sec=stamp[0], nanosec=stamp[1])
        self.tf_odom_obj.child_frame_id = "odom"
        self.tf_odom_obj.transform.translation.x = odom_val[0]
        self.tf_odom_obj.transform.translation.y = odom_val[1]
        self.tf_odom_obj.transform.rotation.x = quat[1]
        self.tf_odom_obj.transform.rotation.y = quat[2]
        self.tf_odom_obj.transform.rotation.z = quat[3]
        self.tf_odom_obj.transform.rotation.w = quat[0]
        self.tf_odom.sendTransform(self.tf_odom_obj)


def main(args=None):
    rclpy.init(args=args)
    driver = HoloDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()