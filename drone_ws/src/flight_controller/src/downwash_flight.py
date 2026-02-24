#!/usr/bin/env python3
"""
Automated flight controller for the downwash scenario.

 - Arms Drone A, takes off to hover altitude via MAVROS.
 - After Drone A is stable, animates Drone B (visual-only Gazebo model)
   along a flyover path directly over Drone A.
 - Loops the flyover so the downwash event repeats.
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3, Pose, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Header

# --------------- configuration ---------------
HOVER_ALT = 5.0          # metres AGL for Drone A
INTRUDER_ALT = 7.0       # metres AGL for Drone B flyover
FLYOVER_SPEED = 2.0      # m/s
FLYOVER_START_X = 12.0   # start X of flyover path
FLYOVER_END_X = -12.0    # end X of flyover path
SETTLE_TIME = 5.0        # seconds to wait after Drone A reaches altitude
LOOP_PAUSE = 4.0         # seconds between consecutive flyovers
ALT_TOLERANCE = 0.5      # metres


class DownwashFlight:
    def __init__(self):
        rospy.init_node('downwash_flight')

        # ---- state bookkeeping ----
        self.mavros_state = State()
        self.current_pose = PoseStamped()

        # ---- subscribers ----
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        # ---- publishers ----
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.model_state_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

        # ---- service proxies (block until available) ----
        rospy.loginfo('Waiting for MAVROS services...')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)  # 20 Hz control loop

    # ---- callbacks ----
    def _state_cb(self, msg):
        self.mavros_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    # ---- helpers ----
    def _wait_for_connection(self):
        rospy.loginfo('Waiting for MAVROS connection...')
        while not rospy.is_shutdown() and not self.mavros_state.connected:
            self.rate.sleep()
        rospy.loginfo('MAVROS connected.')

    def _send_setpoints_preflight(self, n=100):
        """MAVROS requires a stream of setpoints before switching to OFFBOARD."""
        pose = self._hover_pose()
        for _ in range(n):
            if rospy.is_shutdown():
                return
            self.setpoint_pub.publish(pose)
            self.rate.sleep()

    def _hover_pose(self):
        pose = PoseStamped()
        pose.header = Header(stamp=rospy.Time.now(), frame_id='map')
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = HOVER_ALT
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        return pose

    def _set_mode(self, mode):
        for _ in range(5):
            if rospy.is_shutdown():
                return False
            if self.mavros_state.mode == mode:
                return True
            try:
                resp = self.mode_srv(SetModeRequest(custom_mode=mode))
                if resp.mode_sent:
                    rospy.loginfo(f'Mode set to {mode}')
                    return True
            except rospy.ServiceException as e:
                rospy.logwarn(f'SetMode service call failed: {e}')
            rospy.sleep(1.0)
        return False

    def _arm(self):
        for _ in range(5):
            if rospy.is_shutdown():
                return False
            if self.mavros_state.armed:
                return True
            try:
                resp = self.arm_srv(CommandBoolRequest(value=True))
                if resp.success:
                    rospy.loginfo('Vehicle armed.')
                    return True
            except rospy.ServiceException as e:
                rospy.logwarn(f'Arming service call failed: {e}')
            rospy.sleep(1.0)
        return False

    def _at_altitude(self):
        return abs(self.current_pose.pose.position.z - HOVER_ALT) < ALT_TOLERANCE

    # ---- intruder animation ----
    def _move_intruder(self, x, y, z):
        msg = ModelState()
        msg.model_name = 'intruder'
        msg.pose = Pose(position=Point(x, y, z),
                        orientation=Quaternion(0, 0, 0, 1))
        msg.twist = Twist(linear=Vector3(0, 0, 0),
                          angular=Vector3(0, 0, 0))
        msg.reference_frame = 'world'
        self.model_state_pub.publish(msg)

    def _flyover(self):
        """Animate Drone B from FLYOVER_START_X to FLYOVER_END_X at constant altitude."""
        distance = abs(FLYOVER_START_X - FLYOVER_END_X)
        duration = distance / FLYOVER_SPEED
        direction = -1.0 if FLYOVER_START_X > FLYOVER_END_X else 1.0

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed >= duration:
                break
            x = FLYOVER_START_X + direction * FLYOVER_SPEED * elapsed
            self._move_intruder(x, 0.0, INTRUDER_ALT)
            # keep publishing hover setpoint so Drone A stays put
            self.setpoint_pub.publish(self._hover_pose())
            self.rate.sleep()

        # park intruder at end position
        self._move_intruder(FLYOVER_END_X, 0.0, INTRUDER_ALT)

    # ---- main sequence ----
    def run(self):
        self._wait_for_connection()

        # pre-flight setpoint stream (needed before GUIDED/OFFBOARD)
        rospy.loginfo('Streaming initial setpoints...')
        self._send_setpoints_preflight()

        # switch to GUIDED mode and arm
        if not self._set_mode('GUIDED'):
            rospy.logerr('Failed to set GUIDED mode. Aborting.')
            return
        if not self._arm():
            rospy.logerr('Failed to arm. Aborting.')
            return

        # climb to hover altitude
        rospy.loginfo(f'Commanding takeoff to {HOVER_ALT} m...')
        while not rospy.is_shutdown() and not self._at_altitude():
            self.setpoint_pub.publish(self._hover_pose())
            self.rate.sleep()
        rospy.loginfo('Drone A at hover altitude.')

        # settle
        rospy.loginfo(f'Settling for {SETTLE_TIME} s...')
        t_settle = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t_settle).to_sec() >= SETTLE_TIME:
                break
            self.setpoint_pub.publish(self._hover_pose())
            self.rate.sleep()

        # flyover loop
        flyover_count = 0
        while not rospy.is_shutdown():
            flyover_count += 1
            rospy.loginfo(f'--- Flyover #{flyover_count} ---')

            # reset intruder to start position
            self._move_intruder(FLYOVER_START_X, 0.0, INTRUDER_ALT)
            rospy.sleep(0.5)

            self._flyover()

            rospy.loginfo(f'Flyover complete. Pausing {LOOP_PAUSE} s...')
            t_pause = rospy.Time.now()
            while not rospy.is_shutdown():
                if (rospy.Time.now() - t_pause).to_sec() >= LOOP_PAUSE:
                    break
                self.setpoint_pub.publish(self._hover_pose())
                self.rate.sleep()


if __name__ == '__main__':
    try:
        DownwashFlight().run()
    except rospy.ROSInterruptException:
        pass
