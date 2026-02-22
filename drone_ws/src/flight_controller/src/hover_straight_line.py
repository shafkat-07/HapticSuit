#!/usr/bin/python

import rospy
import sys
import time
import enum
import numpy as np

from freyja_msgs.msg import WaypointTarget
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped

class drone_state(enum.Enum):
  WAITING           = 0
  MOVING_FORWARD    = 1
  SNAPPING_BACK     = 2

class FlightPathControllerNode:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('flight_path_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Get the moving and waiting time
        self.waiting_time = rospy.get_param('/{}/waiting_time'.format(rospy.get_name()), 10) 

        # Set the rate
        self.rate = 20 #Hz
        self.dt = 1.0 / self.rate

        # Holds the drone position
        self.drone_position = np.zeros(3)

        # Init the drone and program state
        self._quit = False

        # Init all the publishers and subscribers
        self.waypoint_pub = rospy.Publisher("/discrete_waypoint_target", WaypointTarget, queue_size=10)

        # Used for debug
        self.drone_pos_sub = rospy.Subscriber("/vicon/SUIT/SUIT", TransformStamped, callback=self._GPS_callback)
        
        # Start the program
        time.sleep(2)
        self.start()

    def _GPS_callback(self, msg):
        self.drone_position[0] = msg.transform.translation.y
        self.drone_position[1] = msg.transform.translation.x 
        self.drone_position[2] = msg.transform.translation.z 

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _send_waypoint(self, x, y, z, vx=0, vy=0, vz=0, vel=0):
        # Create the waypoint
        msg = WaypointTarget()
        msg.terminal_pn = x
        msg.terminal_pe = y
        msg.terminal_pd = -z
        msg.terminal_vn = vx
        msg.terminal_ve = vy
        msg.terminal_vd = vz
        msg.terminal_yaw = 0.0

        msg.allocated_time = 0.0
        msg.translational_speed = vel

        msg.waypoint_mode = 1 # It is in speed mode

        self.waypoint_pub.publish(msg)
        self._log("Waypoint sent")

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        # Variable to keep track which direction the drone is moving
        sent_command = False

        # Keep track of time
        time_counter = 0

        # Declare the position and velocity variables
        x_pos = -1
        y_pos = 0
        z_pos = 1
        x_vel = 0
        y_vel = 0
        z_vel = 0

        # Declare the current state
        current_state = drone_state.WAITING

        # Send the initial waypoint
        self._send_waypoint(x=x_pos, y=y_pos, z=z_pos, vx=x_vel, vy=y_vel, vz=z_vel)

        while not self._quit:
            
            # For the waiting state
            if current_state == drone_state.WAITING:
                # The send command
                if not sent_command:
                    self._log("WAITING State")
                    x_pos = -1
                    y_pos = 0
                    z_pos = 1
                    x_vel = 0
                    y_vel = 0
                    z_vel = 0
                    vel   = 1.5
                    # Send the initial waypoint
                    self._send_waypoint(x=x_pos, y=y_pos, z=z_pos, vx=x_vel, vy=y_vel, vz=z_vel, vel=vel)
                    sent_command = True
                # Check the time
                time_counter += 1

                self._log("Count down time: {}".format(np.round(self.waiting_time-(time_counter * self.dt),0)))

                # Once the wait time is done, change directions
                if time_counter * self.dt > self.waiting_time:
                    time_counter = 0
                    current_state = drone_state.MOVING_FORWARD
                    sent_command = False

            # The moving state
            if current_state == drone_state.MOVING_FORWARD:
                # The send command
                if not sent_command:
                    self._log("MOVING FORWARD State")
                    x_pos = 1.5
                    y_pos = 0
                    z_pos = 1
                    x_vel = 1
                    y_vel = 0
                    z_vel = 0
                    vel   = 1.5
                    # Send the initial waypoint
                    self._send_waypoint(x=x_pos, y=y_pos, z=z_pos, vx=x_vel, vy=y_vel, vz=z_vel, vel=vel)
                    sent_command = True
                # The drones position
                if self.drone_position[0] >= 1:
                    current_state = drone_state.SNAPPING_BACK
                    sent_command = False

            # Moving forward
            if current_state == drone_state.SNAPPING_BACK:
                # The send command
                if not sent_command:
                    self._log("SNAPPING BACKWARDS State")
                    x_pos = -1
                    y_pos = 0
                    z_pos = 1
                    x_vel = -3
                    y_vel = 0
                    z_vel = 0
                    vel   = 2
                    # Send the initial waypoint
                    self._send_waypoint(x=x_pos, y=y_pos, z=z_pos, vx=x_vel, vy=y_vel, vz=z_vel, vel=vel)
                    sent_command = True
                # The drones position
                if self.drone_position[0] <= -1:
                    current_state = drone_state.WAITING
                    sent_command = False

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
