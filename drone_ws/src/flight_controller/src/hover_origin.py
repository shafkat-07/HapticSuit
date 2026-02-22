#!/usr/bin/python

import rospy
import sys
import time
import numpy as np

from freyja_msgs.msg import ReferenceState
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped

class FlightPathControllerNode:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('flight_path_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 4 #Hz
        self.dt = 1.0 / self.rate

        # Define the height you want to hover at
        height = rospy.get_param('/{}/desired_height'.format(rospy.get_name()), 1)
        self.hover_height = height

        # Init the drone and program state
        self._quit = False

        # Init all the publishers and subscribers
        self.reference_pub = rospy.Publisher("/reference_state", ReferenceState, queue_size=10)

        # Used for debug
        self.drone_pos_sub = rospy.Subscriber("/vicon/JOZI/JOZI", TransformStamped, callback=self._GPS_callback)
        self.debug_reference = rospy.Publisher("/debug/reference_state", PointStamped, queue_size=10)
        self.debug_drone_position = rospy.Publisher("/debug/drone_position", PointStamped, queue_size=10)
        
        # Start the program
        time.sleep(2)
        self.start()

    def _GPS_callback(self, msg):
        new_msg = PointStamped()
        new_msg.header.frame_id = "world"
        new_msg.point.x = msg.transform.translation.x 
        new_msg.point.y = msg.transform.translation.y 
        new_msg.point.z = msg.transform.translation.z 
        self.debug_drone_position.publish(new_msg)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _send_waypoint(self, x, y, z):
        msg = ReferenceState()
        msg.pn = x
        msg.pe = y
        msg.pd = -z
        msg.vn = 0.0
        msg.ve = 0.0
        msg.vd = 0.0
        msg.yaw = 0.0
        msg.an = 0.0
        msg.ae = 0.0
        msg.ad = 0.0

        self.reference_pub.publish(msg)
        self._log("Waypoint sent")

        # Create the debug msg
        new_msg = PointStamped()
        new_msg.header.frame_id = "world"
        new_msg.point.x = y
        new_msg.point.y = x
        new_msg.point.z = z
        self.debug_reference.publish(new_msg)

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        # Back and fourth counter
        back_fourth_counter = 0
        landing_counter = 0

        sent = False
        # Start with a 5 seconds wait
        counter = -self.rate * 5
        x_pos = 0
        y_pos = 0
        z_pos = self.hover_height

        while not self._quit:
        
            # Send the waypoint
            self._send_waypoint(x=x_pos, y=y_pos, z=z_pos)

            # This just helps wait 10 seconds
            counter += 1

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = FlightPathControllerNode()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
