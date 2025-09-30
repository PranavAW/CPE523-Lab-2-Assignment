#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive



class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.01
        # TODO: create ROS subscribers and publishers.
        self.odom = self.create_subscription(Odometry,'ego_racecar/odom',self.odom_callback,10)
        self.scan = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        
        
        self.drive =   self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # iTTC parameters/state
        self.ttc_threshold = 1.0  # [s] brake when any iTTC < threshold
        self.prev_ranges = None
        self.prev_stamp = None
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        #odommsg = Odometry()
        self.speed = odom_msg.twist.twist.linear.x
        pass

    def scan_callback(self, scan_msg):
        ack_msg = AckermannDriveStamped()
        # Calculate TTC
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)  # instantaneous range r
        n = ranges.size
        if n == 0:
            return

        # Build angles precisely from min + i*increment
        angles = scan_msg.angle_min + np.arange(n, dtype=np.float32) * scan_msg.angle_increment

        # TODO: rdot_proj = - v_x * cos(theta_i)
        # = ...
        rdot_proj = (-1 * self.speed)* np.cos(angles)

        # TODO: iTTC_proj = r / { -rdot_proj }_+  where denom_proj = {x}_+ = max(x, 0)
        denom_proj = np.maximum(np.float32(-rdot_proj),0)
        
        with np.errstate(divide='ignore', invalid='ignore'):
            ittc_proj = np.where(denom_proj > 1e-6, ranges / denom_proj, np.inf)
            ittc = ittc_proj


        # Update memory for next finite-difference step
        self.prev_ranges = ranges.copy()
        self.prev_stamp = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # TODO: Decide to brake if any iTTC is below threshold
        min_ittc = float(np.nanmin(ittc_proj))

        # TODO: publish command to brake
        self.get_logger().info('Subscribing: current speed = "%s"' % self.speed)
        self.get_logger().info('Subscribing: current iTTC = "%s"' % min_ittc)
        
        #if min_ittc > .8:
        #    ack_msg.drive.speed = self.speed
        #    self.drive.publish(ack_msg)
        if min_ittc < self.ttc_threshold:
            ack_msg.drive.speed = 0.0
            ack_msg.drive.acceleration = -5.0
            self.drive.publish(ack_msg)
            

        self.get_logger().info('Publishing: speed = "%s"' % ack_msg.drive.speed)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
