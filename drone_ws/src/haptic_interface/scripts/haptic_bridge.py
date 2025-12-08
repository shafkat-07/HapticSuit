#!/usr/bin/env python3  
import rospy  
import socket  
import time  
from sensor_msgs.msg import Imu

# --- CONFIGURATION ---  
# IP Address of the ESP32 on your Haptic Suit  
SUIT_IP = "192.168.1.100"   
SUIT_PORT = 8888  
# Mass of the drone in Gazebo (Iris is approx 1.5kg)  
DRONE_MASS = 1.5 

class HapticBridge:  
    def __init__(self):  
        rospy.init_node('haptic_bridge')  
          
        # 1. Setup UDP Connection to Suit  
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
          
        # 2. Subscribe to IMU Data  
        # /mavros/imu/data contains the raw acceleration felt by the drone  
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)  
          
        rospy.loginfo(f"Haptic Bridge Initialized. Sending to {SUIT_IP}:{SUIT_PORT}")

    def imu_cb(self, msg):  
        """  
        Runs ~50 times per second.   
        Reads simulated physics -> Sends commands to physical suit.  
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
        if abs(force_x) < 0.2: force_x = 0  
        if abs(force_y) < 0.2: force_y = 0  
        if abs(force_z) < 0.2: force_z = 0

        # D. Format Packet  
        # Format: "fx,fy,fz" (e.g., "1.2,-0.5,0.0")  
        packet = f"{force_x:.2f},{force_y:.2f},{force_z:.2f}"  
          
        # E. Send to Suit  
        try:  
            self.sock.sendto(packet.encode(), (SUIT_IP, SUIT_PORT))  
              
            # Debugging: Only print if there is significant wind/force  
            if abs(force_x) > 1.0 or abs(force_y) > 1.0:  
                rospy.loginfo(f"TURBULENCE DETECTED! Force: {packet}")  
                  
        except Exception as e:  
            rospy.logwarn(f"UDP Error: {e}")

if __name__ == '__main__':  
    try:  
        bridge = HapticBridge()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        pass
