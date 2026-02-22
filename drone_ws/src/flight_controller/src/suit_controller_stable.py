#!/usr/bin/python

import rospy
import sys
import time
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
        self.rate = 0.5
        self.dt = 1.0 / self.rate

        # Get the desired throttle
        throttle_param = rospy.get_param('/{}/desired_throttle'.format(rospy.get_name()), 0)

        # Define the current throttle and angle
        self.throttle   = [throttle_param, throttle_param]
        self.angle      = [90, -90]
        # self.angle      = [180, -180]

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

        # While we are running the program
        while not self._quit:
            
            # Send the command
            self._send_command(self.angle[0], self.angle[1], self.throttle[0], self.throttle[1])

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