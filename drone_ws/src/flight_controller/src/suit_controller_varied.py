#!/usr/bin/python

import rospy
import sys
import time
import random
import socket
import numpy as np

from freyja_msgs.msg import WaypointTarget
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray

class SuitController:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('suit_controller')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Declare the socket variables
        self.UDP_IP = "192.168.3.10"
        self.UDP_PORT = 8888

        # Init the drone and program state
        self._quit = False

        # Set the rate
        self.rate = 5
        self.dt = 1.0 / self.rate

        # Define the current throttle and angle
        self.throttle   = [0, 0]
        self.angle      = [90, -90]

        # Create the distribution
        # Setting 1
        # mu, sigma = 8, 1.5
        # self.throttle_values = np.random.normal(mu, sigma, 10000)
        # mu, sigma = 0, 1
        # self.throttle_variation = np.random.normal(mu, sigma, 10000)
        # mu, sigma = 0, 3
        # self.angle_variation = np.random.normal(mu, sigma, 10000)
        # self.time_variation =np.full(1, 10000)

        # Setting 2
        # mu, sigma = 10, 2
        # self.throttle_values = np.random.normal(mu, sigma, 10000)
        # mu, sigma = 0, 2
        # self.throttle_variation = np.random.normal(mu, sigma, 10000)
        # mu, sigma = 0, 5
        # self.angle_variation = np.random.normal(mu, sigma, 10000)
        # mu, sigma = 0, 3
        # self.time_variation = np.random.normal(mu, sigma, 10000)

        # Setting 3
        mu, sigma = 10, 4
        self.throttle_values = np.random.normal(mu, sigma, 10000)
        mu, sigma = 0, 1
        self.throttle_variation = np.random.normal(mu, sigma, 10000)
        mu, sigma = 0, 8
        self.angle_variation = np.random.normal(mu, sigma, 10000)
        mu, sigma = 0, 3
        self.time_variation = np.random.normal(mu, sigma, 10000)

        # Set the drone current position
        self.drone_pos = np.zeros(2)
        self.current_waypoint = np.zeros(2)

        # Init all the publishers and subscribers
        self.waypoint_sub = rospy.Subscriber("/discrete_waypoint_target", WaypointTarget, callback=self._waypoint_callback)
        self.drone_pos_sub = rospy.Subscriber("/vicon/SUIT/SUIT", TransformStamped, callback=self._GPS_callback)
        self.suit_control_pub = rospy.Publisher("/haptic_suit/control_signal", Float64MultiArray, queue_size=10)
        
        # Start the program
        time.sleep(2)
        self.start()

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        rospy.loginfo(str(rospy.get_name()) + ": " + str(msg))

    def _GPS_callback(self, msg):
        self.drone_pos[0] = msg.transform.translation.x 
        self.drone_pos[1] = msg.transform.translation.y 

    def _waypoint_callback(self, msg):
        self.current_waypoint[0] = msg.terminal_pe
        self.current_waypoint[1] = msg.terminal_pn

    def _send_command(self, angle1, angle2, throttle1, throttle2):
        self._log("Sent: a1:{}  a2:{}  t1:{}  t2:{}".format(angle1, angle2, throttle1, throttle2))
        msg = Float64MultiArray()
        msg.data = [angle1, angle2, throttle1, throttle2] 
        self.suit_control_pub.publish(msg)
        # Send the command over the socket
        socket_msg = "{}, {}, {}, {}".format(angle1, angle2, throttle1, throttle2)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(socket_msg, (self.UDP_IP, self.UDP_PORT))

    def _mainloop(self):
        # Set the rate
        r = rospy.Rate(self.rate)

        counter = 0

        a1 = self.angle[0]
        a2 = self.angle[1]
        t1 = self.throttle[0]
        t2 = self.throttle[1]

        k = random.randrange(0, 9999)
        wait_time = self.time_variation[k]

        # While we are running the program
        while not self._quit:

            counter += 1

            if counter >= wait_time:
                # Compute the values
                j = random.randrange(0, 9999)
                a1 = self.angle[0] + self.angle_variation[j]
                j = random.randrange(0, 9999)
                a2 = self.angle[1] + self.angle_variation[j]
            
                i = random.randrange(0, 9999)
                j = random.randrange(0, 9999)
                t1 = self.throttle[0] + self.throttle_values[i] + self.throttle_variation[j]
                j = random.randrange(0, 9999)
                t2 = self.throttle[1] + self.throttle_values[i] + self.throttle_variation[j]
                
                # Reset counter
                counter = 0
                k = random.randrange(0, 9999)
                wait_time = self.time_variation[k]

            # Clip the values
            a1 = np.clip(a1, 70, 110)
            a2 = np.clip(a2, -110, -70)
            t1 = np.clip(t1, 0, 25)
            t2 = np.clip(t2, 0, 25)
            
            # Send the command
            self._send_command(a1, a2, t1, t2)
            print("--------")

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = SuitController()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)