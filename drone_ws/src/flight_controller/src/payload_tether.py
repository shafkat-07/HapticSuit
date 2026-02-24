#!/usr/bin/env python3
"""
Rope-like tether between the drone and suspended payload.

Instead of a world-level ball joint (unreliable between separate models
in Gazebo classic / ODE), this node models the tether as a spring-damper
that applies tension only when the distance exceeds TETHER_LENGTH.
When slack the payload is free to sit on the ground or move under
gravity and wind alone.

Forces are applied via /gazebo/apply_body_wrench -- the same proven
pattern used by downwash_force.py.
"""

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3, Point

# ======================== tunable parameters ========================

TETHER_LENGTH = 0.5     # m   rope length (slack below this distance)
SPRING_K = 500.0        # N/m stiffness when taut
DAMPING_C = 15.0        # N·s/m  damping along tether axis
MAX_FORCE = 30.0        # N   safety clamp

RATE_HZ = 50
WRENCH_DURATION_S = 0.025  # slightly > 1/RATE_HZ to avoid force gaps

# Gazebo model names (as they appear in /gazebo/model_states)
DRONE_MODEL = "iris_demo"
PAYLOAD_MODEL = "suspended_payload"

# Fully-qualified link names (for apply_body_wrench)
DRONE_BODY = "iris_demo::iris::base_link"
PAYLOAD_BODY = "suspended_payload::link"


class PayloadTether:
    def __init__(self):
        rospy.init_node('payload_tether')

        self.drone_pos = None
        self.drone_vel = None
        self.payload_pos = None
        self.payload_vel = None

        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._states_cb)

        rospy.loginfo('Tether: waiting for /gazebo/apply_body_wrench ...')
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.wrench_srv = rospy.ServiceProxy(
            '/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.rate = rospy.Rate(RATE_HZ)
        rospy.loginfo(
            f'Tether ready: length={TETHER_LENGTH}m, '
            f'K={SPRING_K}, C={DAMPING_C}')

    # ----------------------------------------------------------------
    def _states_cb(self, msg):
        for i, name in enumerate(msg.name):
            p = msg.pose[i].position
            v = msg.twist[i].linear
            if name == DRONE_MODEL:
                self.drone_pos = np.array([p.x, p.y, p.z])
                self.drone_vel = np.array([v.x, v.y, v.z])
            elif name == PAYLOAD_MODEL:
                self.payload_pos = np.array([p.x, p.y, p.z])
                self.payload_vel = np.array([v.x, v.y, v.z])

    # ----------------------------------------------------------------
    def _apply_wrench(self, body_name, force):
        wrench = Wrench()
        wrench.force = Vector3(*force)
        wrench.torque = Vector3(0, 0, 0)
        try:
            self.wrench_srv(
                body_name=body_name,
                reference_frame='world',
                reference_point=Point(0, 0, 0),
                wrench=wrench,
                start_time=rospy.Time(0),
                duration=rospy.Duration(WRENCH_DURATION_S))
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f'Tether wrench failed: {e}')

    # ----------------------------------------------------------------
    def run(self):
        while not rospy.is_shutdown():
            if (self.drone_pos is None or self.payload_pos is None or
                    self.drone_vel is None or self.payload_vel is None):
                self.rate.sleep()
                continue

            displacement = self.drone_pos - self.payload_pos
            distance = np.linalg.norm(displacement)

            if distance < 1e-6 or distance <= TETHER_LENGTH:
                self.rate.sleep()
                continue

            direction = displacement / distance   # unit: payload -> drone
            extension = distance - TETHER_LENGTH

            spring_force = SPRING_K * extension

            relative_vel = self.drone_vel - self.payload_vel
            vel_along = np.dot(relative_vel, direction)
            damping_force = DAMPING_C * vel_along

            tension = max(0.0, spring_force + damping_force)
            tension = min(tension, MAX_FORCE)

            force_on_payload = (tension * direction).tolist()
            force_on_drone = (-tension * direction).tolist()

            self._apply_wrench(PAYLOAD_BODY, force_on_payload)
            self._apply_wrench(DRONE_BODY, force_on_drone)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        PayloadTether().run()
    except rospy.ROSInterruptException:
        pass
