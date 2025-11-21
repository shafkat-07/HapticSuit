#!/usr/bin/env python

"""
F450 MAVROS Keyboard Teleop
Controls the F450 drone in Gazebo SITL using MAVROS
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import sys
import select
import termios
import tty

INSTRUCTIONS = """
--------------------------------------------------
F450 MAVROS Keyboard Teleop
--------------------------------------------------
Controls:
   w/s : forward/backward
   a/d : left/right
   i/k : up/down
   j/l : rotate left/right

Commands:
   t   : Takeoff to 2 meters
   g   : Land
   m   : Set mode to GUIDED
   b   : Arm drone
   
Emergency:
   SPACE : hover (zero velocity)

Exit:
   CTRL+C : quit
--------------------------------------------------
"""

class MavrosKeyboardTeleop:
    def __init__(self):
        rospy.init_node('f450_mavros_teleop', anonymous=True)

        self.current_state = None
        self.current_pose = None
        
        # Subscribers
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        # Service clients
        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        
        # Speed settings
        self.linear_speed = 1.0
        self.angular_speed = 0.5
        self.vertical_speed = 0.5
        # Terminal settings (simplified - no raw mode needed)
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Rate
        self.rate = rospy.Rate(20) # Must be faster than 2Hz for offboard mode

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def get_key(self, timeout=0.1):
        """Get a single keypress from the terminal"""
        # Use select to check if input is available
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            rospy.loginfo("Key detected: '{}'".format(repr(key)))
            return key
        return ''

    def takeoff(self, altitude):
        if self.current_pose is None:
            rospy.logwarn("Cannot takeoff without a current pose estimate.")
            return

        if not self.current_state.armed:
            rospy.logwarn("Drone is not armed. Please arm first.")
            return

        if self.current_state.mode != "GUIDED":
            rospy.logwarn("Drone is not in GUIDED mode. Please set mode first.")
            return

        rospy.loginfo("Taking off to {}m".format(altitude))
        
        # Send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.current_pose)
            self.rate.sleep()

        t_start = rospy.Time.now()
        while (rospy.Time.now() - t_start) < rospy.Duration(5.0):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = self.current_pose.pose.position.x
            pose.pose.position.y = self.current_pose.pose.position.y
            pose.pose.position.z = altitude
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        rospy.loginfo("Takeoff complete. Switching to velocity control.")


    def run(self):
        """Main control loop"""
        
        # Wait for FCU connection
        while not rospy.is_shutdown() and self.current_state is None:
            rospy.loginfo("Waiting for FCU connection...")
            self.rate.sleep()

        rospy.loginfo("FCU connected.")

        # Set terminal to unbuffered mode for immediate key capture
        tty.setcbreak(sys.stdin.fileno())

        try:
            print(INSTRUCTIONS)
            rospy.loginfo("Keyboard control active - press keys now!")
            while not rospy.is_shutdown():
                key = self.get_key()
                
                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.linear.y = self.linear_speed
                elif key == 'd':
                    twist.linear.y = -self.linear_speed
                elif key == 'i':
                    twist.linear.z = self.vertical_speed
                elif key == 'k':
                    twist.linear.z = -self.vertical_speed
                elif key == 'j':
                    twist.angular.z = self.angular_speed
                elif key == 'l':
                    twist.angular.z = -self.angular_speed
                elif key == ' ': # Hover
                    pass # Zero twist is hover
                elif key == 'b': # Arm
                    if not self.current_state.armed:
                        self.arming_client(True)
                        rospy.loginfo("Drone armed")
                elif key == 'm': # Set mode to GUIDED
                    self.set_mode_client(custom_mode='GUIDED')
                    rospy.loginfo("Mode set to GUIDED")
                elif key == 't': # Takeoff
                    self.takeoff(2.0)
                elif key == 'g': # Land
                    self.set_mode_client(custom_mode='LAND')
                    rospy.loginfo("Landing...")
                elif key == '\x03': # CTRL+C
                    break

                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()

        except Exception as e:
            print(e)
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


if __name__ == '__main__':
    try:
        teleop = MavrosKeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

