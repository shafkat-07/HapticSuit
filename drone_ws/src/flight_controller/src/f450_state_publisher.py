#!/usr/bin/env python

"""
F450 State Publisher Node (Hector Version)
Aggregates all drone sensor data and publishes comprehensive state information
Works with Hector Gazebo plugins for better GPS and flight dynamics
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Float64
import tf
import math

class F450StatePublisher:
    def __init__(self):
        rospy.init_node('f450_state_publisher', anonymous=True)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/f450/ground_truth/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/f450/imu/data', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/f450/gps/fix', NavSatFix, self.gps_callback)
        
        # Publishers - Individual state components
        self.pose_pub = rospy.Publisher('/f450/state/pose', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/f450/state/velocity', TwistStamped, queue_size=10)
        self.altitude_pub = rospy.Publisher('/f450/state/altitude', Float64, queue_size=10)
        self.attitude_pub = rospy.Publisher('/f450/state/attitude', Vector3Stamped, queue_size=10)
        self.gps_pub = rospy.Publisher('/f450/state/gps', NavSatFix, queue_size=10)
        
        # State variables
        self.current_odom = None
        self.current_imu = None
        self.current_gps = None
        
        # Publishing rate
        self.rate = rospy.Rate(50)  # 50 Hz
        
        rospy.loginfo("F450 State Publisher (Hector) initialized")
        rospy.loginfo("Publishing state on the following topics:")
        rospy.loginfo("  - /f450/state/pose        : Position and orientation")
        rospy.loginfo("  - /f450/state/velocity    : Linear and angular velocities")
        rospy.loginfo("  - /f450/state/altitude    : Altitude (Z position)")
        rospy.loginfo("  - /f450/state/attitude    : Roll, Pitch, Yaw angles")
        rospy.loginfo("  - /f450/state/gps         : GPS coordinates (from Hector plugin)")
        
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.current_odom = msg
        
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.current_imu = msg
        
    def gps_callback(self, msg):
        """Callback for GPS data from Hector plugin"""
        self.current_gps = msg
        
    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        euler = tf.transformations.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ])
        return euler  # Returns (roll, pitch, yaw)
    
    def publish_state(self):
        """Publish aggregated state information"""
        if self.current_odom is None:
            return
            
        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose = self.current_odom.pose.pose
        self.pose_pub.publish(pose_msg)
        
        # Publish Velocity
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.header.frame_id = "base_link"
        velocity_msg.twist = self.current_odom.twist.twist
        self.velocity_pub.publish(velocity_msg)
        
        # Publish Altitude
        altitude_msg = Float64()
        altitude_msg.data = self.current_odom.pose.pose.position.z
        self.altitude_pub.publish(altitude_msg)
        
        # Publish Attitude (Roll, Pitch, Yaw)
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = rospy.Time.now()
        attitude_msg.header.frame_id = "base_link"
        
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(self.current_odom.pose.pose.orientation)
        attitude_msg.vector.x = roll
        attitude_msg.vector.y = pitch
        attitude_msg.vector.z = yaw
        self.attitude_pub.publish(attitude_msg)
        
        # Publish GPS (from Hector plugin - real GPS simulation!)
        if self.current_gps is not None:
            self.gps_pub.publish(self.current_gps)
    
    def run(self):
        """Main publishing loop"""
        rospy.loginfo("F450 State Publisher running...")
        
        while not rospy.is_shutdown():
            self.publish_state()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = F450StatePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass

