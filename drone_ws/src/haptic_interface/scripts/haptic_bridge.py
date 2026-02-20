#!/usr/bin/env python3  
import rospy  
import socket  
import time
import sys
import os
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

# Add current directory to path so we can import local modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from kinematics_solver import KinematicsSolver

# --- CONFIGURATION ---  
# IP Address for Simulation (Localhost)
SUIT_IP = "127.0.0.1"   
SUIT_PORT = 8888  
# Mass of the drone in Gazebo (Iris is approx 1.5kg)  
DRONE_MASS = 1.5 

class HapticBridge:  
    def __init__(self):  
        rospy.init_node('haptic_bridge')  
          
        # 1. Setup UDP Connection to Suit  
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 2. Setup Kinematics Solver
        # Using default ky=2.0, km=3.0 as per matlab files
        self.solver = KinematicsSolver(ky=2.0, km=3.0)
          
        # 3. Subscribe to IMU Data  
        # /mavros/imu/data contains the raw acceleration felt by the drone  
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        
        # 4. Debug Publisher
        # Publishes [Fx, Fy, Fz, w1, w2, w3, w4] for visualization
        self.pub_debug = rospy.Publisher('~debug', Float64MultiArray, queue_size=10)
          
        rospy.loginfo(f"Haptic Bridge Initialized. Sending to {SUIT_IP}:{SUIT_PORT}")
        rospy.loginfo("Mode: Simulation (Kinematics Solver Enabled)")

    def imu_cb(self, msg):  
        """  
        Runs ~50 times per second.   
        Reads simulated physics -> Solves IK -> Sends commands to physical suit.  
        """  
        # A. Read Acceleration from Gazebo (m/s^2)  
        ax = msg.linear_acceleration.x  
        ay = msg.linear_acceleration.y  
        az = msg.linear_acceleration.z  
          
        # B. Calculate Force (F = ma)  
        # Note: We subtract 9.81 from Z to remove gravity's constant pull  
        # for a "deviation" based force.  
        force_x = ax * DRONE_MASS  
        force_y = ay * DRONE_MASS  
        force_z = (az - 9.81) * DRONE_MASS 

        # C. Simple Noise Filter (Deadzone)  
        # Ignores tiny jitters in the simulation  
        if abs(force_x) < 0.2: force_x = 0.0  
        if abs(force_y) < 0.2: force_y = 0.0  
        if abs(force_z) < 0.2: force_z = 0.0

        # D. Solve Inverse Kinematics
        # target_forces = [Fx, Fy, Fz]
        solution = self.solver.solve([force_x, force_y, force_z])
        
        # Solution: [w1, w2, w3, w4, q1, q2, q3, q4]
        w = solution[0:4] # Speeds/Throttles
        q = solution[4:8] # Angles in Radians
        
        # Convert Angles to Degrees for display/firmware
        q_deg = np.degrees(q)

        # E. Format Packet  
        # Format: "q1,q2,q3,q4,w1,w2,w3,w4" (CSV)
        # q in degrees, w as is (0-100)
        packet = f"{q_deg[0]:.1f},{q_deg[1]:.1f},{q_deg[2]:.1f},{q_deg[3]:.1f},{w[0]:.1f},{w[1]:.1f},{w[2]:.1f},{w[3]:.1f}"
          
        # F. Send to Suit  
        try:  
            self.sock.sendto(packet.encode(), (SUIT_IP, SUIT_PORT))  
            
            # Publish Debug Data for rqt_plot
            # [Fx, Fy, Fz, w1, w2, w3, w4]
            debug_msg = Float64MultiArray()
            debug_msg.data = [force_x, force_y, force_z, w[0], w[1], w[2], w[3]]
            self.pub_debug.publish(debug_msg)
              
            # Debugging: Only print if there is significant wind/force  
            if abs(force_x) > 1.0 or abs(force_y) > 1.0:  
                rospy.loginfo(f"TURBULENCE! F: [{force_x:.1f}, {force_y:.1f}, {force_z:.1f}] -> Motors: {packet}")  
                  
        except Exception as e:  
            rospy.logwarn(f"UDP Error: {e}")

if __name__ == '__main__':  
    try:  
        bridge = HapticBridge()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        pass
