#!/usr/bin/python

import rospy
import sys
import time
import random
import socket
import numpy as np

# Import the cart simulation
from cart import *
from PID import *

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

        # Get the desired throttle
        throttle_param = rospy.get_param('/{}/desired_throttle'.format(rospy.get_name()), 0)

        # Init the drone and program state
        self._quit = False

        # Set the rate
        self.rate = 50
        self.dt = 1.0 / self.rate

        # Define the current throttle and angle
        self.throttle   = [throttle_param, throttle_param]
        self.angle      = [-180, 180]

        # Set the drone current position
        self.drone_pos = np.zeros(3)
        
        # Init the simulation variables
        pole_length = 0.6
        masscart    = 2
        masspole    = 0.3

        # Create the simulation
        self.env = CartPoleEnv(timestep=self.dt, pole_length=pole_length, masscart=masscart, masspole=masspole)
        observation = self.env.reset()

        # Create the control PID that matches the cart to the drone
        self.pid = PID(Kp_in=120.0, Ki_in=-0, Kd_in=15.0, rate_in=self.rate)

        # Let the simulation settle
        for i in range(100):
            # Render the environment
            self.env.render()
            observation, reward, done, info = self.env.step(0, 0)

        # Init all the publishers and subscribers
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
        self.drone_pos[0] = msg.transform.translation.y 
        self.drone_pos[1] = msg.transform.translation.x 
        self.drone_pos[1] = msg.transform.translation.z 

    def _send_command(self, angle1, angle2, throttle1, throttle2):
        # self._log("Sent: a1:{}  a2:{}  t1:{}  t2:{}".format(angle1, angle2, throttle1, throttle2))
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

        # Create the throttle and angle messages
        a1 = self.angle[0]
        a2 = self.angle[1]
        t1 = self.throttle[0]
        t2 = self.throttle[1]

        # Create the cart details
        cart_x_pos = 0

        # Command counter
        com_counter = 0

        # While we are running the program
        while not self._quit:

            # Increment the counter
            com_counter += 1

            # Render the environment
            self.env.render()

            # Get the drones current x position
            drone_x_pos = self.drone_pos[0]

            # Use a PID to match the cart to the drone position
            output = self.pid.get_output(drone_x_pos, cart_x_pos)
            
            # Simulate
            observation, reward, done, info = self.env.step(output, drone_x_pos)
            
            # Get the pendulum angle and cart position
            pendulum_angle = np.clip(np.rad2deg(observation[2]) - 180, -60, 60)
            cart_x_pos = observation[0]
            
            # Send the command
            if com_counter > 5:
                a1 = self.angle[0] - pendulum_angle
                a2 = self.angle[1] + pendulum_angle
                self._send_command(a1, a2, t1, t2)
                com_counter = 0

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