#!/usr/bin/env python3
"""
Automated flight controller for the wind + payload scenario.

Arms the drone, takes off to 2 m, flies 10 m in the +X direction
(with constant wind), then lands.  No manual input required.
"""

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import HomePosition, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Header

# --------------- configuration ---------------
HOVER_ALT = 2.0          # metres AGL
CRUISE_SPEED = 1.5       # m/s along +X
CRUISE_DISTANCE = 10.0   # metres
SETTLE_TIME = 5.0        # seconds to stabilise after reaching altitude / waypoint
ALT_TOLERANCE = 0.3      # metres
POS_TOLERANCE = 0.5      # metres – close-enough to waypoint
RATE_HZ = 20


class WindPayloadFlight:
    def __init__(self):
        rospy.init_node('wind_payload_flight')

        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.home_received = False

        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self._home_cb)

        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.loginfo('Waiting for MAVROS services...')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(RATE_HZ)

    # ---- callbacks ----
    def _state_cb(self, msg):
        self.mavros_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _home_cb(self, msg):
        if not self.home_received:
            rospy.loginfo('Home position received.')
            self.home_received = True

    # ---- helpers ----
    def _wait_for_connection(self):
        rospy.loginfo('Waiting for MAVROS connection...')
        while not rospy.is_shutdown() and not self.mavros_state.connected:
            self.rate.sleep()
        rospy.loginfo('MAVROS connected.')

    def _wait_for_ekf(self, timeout=90.0):
        """Block until ArduPilot sets a home position (EKF converged)."""
        rospy.loginfo('Waiting for EKF convergence (home position)...')
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not self.home_received:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr('Timed out waiting for home position.')
                return False
            self.setpoint_pub.publish(self._make_pose(0, 0, HOVER_ALT))
            self.rate.sleep()
        return True

    def _send_setpoints_preflight(self, n=100):
        """MAVROS requires a stream of setpoints before switching to GUIDED."""
        pose = self._make_pose(0.0, 0.0, HOVER_ALT)
        for _ in range(n):
            if rospy.is_shutdown():
                return
            self.setpoint_pub.publish(pose)
            self.rate.sleep()

    def _make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header = Header(stamp=rospy.Time.now(), frame_id='map')
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
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
                rospy.logwarn(f'SetMode failed: {e}')
            rospy.sleep(1.0)
        return False

    def _arm(self):
        for _ in range(15):
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
                rospy.logwarn(f'Arming failed: {e}')
            rospy.sleep(1.0)
        return False

    def _at_position(self, x, y, z):
        p = self.current_pose.pose.position
        dx = p.x - x
        dy = p.y - y
        dz = p.z - z
        return (dx*dx + dy*dy + dz*dz) ** 0.5 < POS_TOLERANCE

    def _at_altitude(self):
        return abs(self.current_pose.pose.position.z - HOVER_ALT) < ALT_TOLERANCE

    def _hold_for(self, seconds, x, y, z):
        """Publish a fixed setpoint for *seconds*."""
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() >= seconds:
                break
            self.setpoint_pub.publish(self._make_pose(x, y, z))
            self.rate.sleep()

    # ---- main sequence ----
    def run(self):
        self._wait_for_connection()

        rospy.loginfo('Streaming initial setpoints...')
        self._send_setpoints_preflight()

        if not self._wait_for_ekf():
            rospy.logerr('EKF did not converge. Aborting.')
            return

        if not self._set_mode('GUIDED'):
            rospy.logerr('Failed to set GUIDED mode. Aborting.')
            return
        if not self._arm():
            rospy.logerr('Failed to arm. Aborting.')
            return

        # --- takeoff ---
        rospy.loginfo(f'Taking off to {HOVER_ALT} m...')
        while not rospy.is_shutdown() and not self._at_altitude():
            self.setpoint_pub.publish(self._make_pose(0.0, 0.0, HOVER_ALT))
            self.rate.sleep()
        rospy.loginfo('At hover altitude.')

        rospy.loginfo(f'Settling for {SETTLE_TIME} s...')
        self._hold_for(SETTLE_TIME, 0.0, 0.0, HOVER_ALT)

        # --- cruise 10 m in +X ---
        rospy.loginfo(f'Flying {CRUISE_DISTANCE} m in +X at {CRUISE_SPEED} m/s...')
        duration = CRUISE_DISTANCE / CRUISE_SPEED
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed >= duration:
                break
            x = CRUISE_SPEED * elapsed
            self.setpoint_pub.publish(self._make_pose(x, 0.0, HOVER_ALT))
            self.rate.sleep()

        # hold at final position until the drone catches up
        rospy.loginfo('Reaching final waypoint...')
        while not rospy.is_shutdown() and not self._at_position(CRUISE_DISTANCE, 0.0, HOVER_ALT):
            self.setpoint_pub.publish(self._make_pose(CRUISE_DISTANCE, 0.0, HOVER_ALT))
            self.rate.sleep()

        rospy.loginfo(f'At destination. Settling for {SETTLE_TIME} s...')
        self._hold_for(SETTLE_TIME, CRUISE_DISTANCE, 0.0, HOVER_ALT)

        # --- land ---
        rospy.loginfo('Switching to LAND mode...')
        if not self._set_mode('LAND'):
            rospy.logerr('Failed to set LAND mode.')
            return

        rospy.loginfo('Landing... waiting for disarm.')
        while not rospy.is_shutdown() and self.mavros_state.armed:
            self.rate.sleep()

        rospy.loginfo('Mission complete. Vehicle disarmed.')


if __name__ == '__main__':
    try:
        WindPayloadFlight().run()
    except rospy.ROSInterruptException:
        pass
