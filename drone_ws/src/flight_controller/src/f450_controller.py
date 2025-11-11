#!/usr/bin/env python3
"""
Simple controller for the F450 quadcopter in Gazebo
Publishes velocity commands to control the drone
"""

import rospy
from geometry_msgs.msg import Twist
import sys

class F450Controller:
    def __init__(self):
        rospy.init_node('f450_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Control rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("F450 Controller initialized")
        
    def hover(self):
        """Make the drone hover in place"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
    def move(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        """
        Move the drone with specified velocities
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s)
            linear_z: Up/down velocity (m/s)
            angular_z: Yaw rate (rad/s)
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
        
    def run_demo(self):
        """Run a simple demo trajectory"""
        rospy.loginfo("Starting demo trajectory...")
        
        # Hover for 2 seconds
        rospy.loginfo("Hovering...")
        for _ in range(20):
            self.hover()
            self.rate.sleep()
        
        # Move forward for 3 seconds
        rospy.loginfo("Moving forward...")
        for _ in range(30):
            self.move(linear_x=0.5)
            self.rate.sleep()
        
        # Hover for 2 seconds
        rospy.loginfo("Hovering...")
        for _ in range(20):
            self.hover()
            self.rate.sleep()
        
        # Move backward for 3 seconds
        rospy.loginfo("Moving backward...")
        for _ in range(30):
            self.move(linear_x=-0.5)
            self.rate.sleep()
        
        # Hover and finish
        rospy.loginfo("Demo complete. Hovering...")
        for _ in range(20):
            self.hover()
            self.rate.sleep()
            
    def spin(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = F450Controller()
        
        # Check if demo mode is requested
        if len(sys.argv) > 1 and sys.argv[1] == 'demo':
            controller.run_demo()
        else:
            rospy.loginfo("F450 Controller running. Publish to /cmd_vel to control the drone.")
            controller.spin()
            
    except rospy.ROSInterruptException:
        pass

