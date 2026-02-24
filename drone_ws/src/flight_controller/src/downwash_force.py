#!/usr/bin/env python3
"""
Downwash force model with Dryden-like turbulence overlay.

Subscribes to /gazebo/model_states, computes the aerodynamic force that
Drone B's rotor wake exerts on Drone A, and applies it via
/gazebo/apply_body_wrench.  A band-limited stochastic turbulence
component is added so the force is not a smooth bell-curve but a
realistic, gusty disturbance.
"""

import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3, Vector3Stamped, Point
from std_msgs.msg import Header

# ======================= tunable parameters =======================

# Intruder (Drone B) hover thrust  ~  m * g
INTRUDER_MASS = 1.5            # kg
GRAVITY = 9.81                 # m/s^2
THRUST = INTRUDER_MASS * GRAVITY

# Rotor disk geometry (Iris-class quad)
ROTOR_RADIUS = 0.25            # m
DISK_AREA = math.pi * ROTOR_RADIUS ** 2

# Atmosphere
AIR_DENSITY = 1.225            # kg/m^3

# Induced velocity at the rotor disk  (momentum theory)
V_INDUCED = math.sqrt(THRUST / (2.0 * AIR_DENSITY * DISK_AREA))

# Drone A effective drag parameters
CD = 1.0                       # bluff-body drag coefficient
A_EFF = 0.04                   # m^2  effective frontal area

# Spatial decay
SIGMA_BASE = 0.30              # m   base lateral spread at disk
SIGMA_RATE = 0.15              # m/m  additional spread per metre below
MAX_HORIZONTAL_RANGE = 3.0     # m   ignore if farther apart horizontally
MAX_VERTICAL_RANGE = 15.0      # m   ignore if too far below

# Turbulence overlay (Dryden-inspired band-limited noise)
TURB_INTENSITY = 0.40          # fraction of base force magnitude
TURB_BANDWIDTH = 5.0           # Hz  -3 dB bandwidth of noise filter
TURB_LATERAL_SCALE = 0.25      # turbulence adds lateral force too

# Node timing
RATE_HZ = 50
WRENCH_DURATION_S = 0.025      # slightly > 1/RATE_HZ to avoid gaps


class DrydenNoise:
    """First-order low-pass filtered white noise (single axis)."""

    def __init__(self, bandwidth, dt):
        self.alpha = 1.0 - math.exp(-2.0 * math.pi * bandwidth * dt)
        self.state = 0.0

    def step(self):
        white = np.random.randn()
        self.state += self.alpha * (white - self.state)
        return self.state


class DownwashForce:
    def __init__(self):
        rospy.init_node('downwash_force')

        self.iris_pos = None
        self.intruder_pos = None

        dt = 1.0 / RATE_HZ
        self.noise_x = DrydenNoise(TURB_BANDWIDTH, dt)
        self.noise_y = DrydenNoise(TURB_BANDWIDTH, dt)
        self.noise_z = DrydenNoise(TURB_BANDWIDTH, dt)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self._states_cb)

        rospy.loginfo('Waiting for /gazebo/apply_body_wrench ...')
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.wrench_srv = rospy.ServiceProxy(
            '/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.debug_pub = rospy.Publisher(
            '/downwash/force', Vector3Stamped, queue_size=10)

        self.rate = rospy.Rate(RATE_HZ)
        rospy.loginfo(
            f'Downwash force node ready  (v_i={V_INDUCED:.2f} m/s, '
            f'turb={TURB_INTENSITY*100:.0f}%)')

    # -----------------------------------------------------------------
    def _states_cb(self, msg):
        for i, name in enumerate(msg.name):
            p = msg.pose[i].position
            if name == 'iris':
                self.iris_pos = np.array([p.x, p.y, p.z])
            elif name == 'intruder':
                self.intruder_pos = np.array([p.x, p.y, p.z])

    # -----------------------------------------------------------------
    def _compute_force(self):
        """Return (Fx, Fy, Fz) in world frame to apply on Drone A."""
        if self.iris_pos is None or self.intruder_pos is None:
            return 0.0, 0.0, 0.0

        dx = self.iris_pos[0] - self.intruder_pos[0]
        dy = self.iris_pos[1] - self.intruder_pos[1]
        dz = self.intruder_pos[2] - self.iris_pos[2]  # positive = B above A

        if dz <= 0:
            return 0.0, 0.0, 0.0

        r_horiz = math.sqrt(dx * dx + dy * dy)
        if r_horiz > MAX_HORIZONTAL_RANGE or dz > MAX_VERTICAL_RANGE:
            return 0.0, 0.0, 0.0

        # --- smooth base downwash velocity (momentum theory + decay) ---
        sigma = SIGMA_BASE + SIGMA_RATE * dz
        radial_decay = math.exp(-(r_horiz ** 2) / (2.0 * sigma ** 2))
        vertical_decay = (ROTOR_RADIUS / max(dz, ROTOR_RADIUS)) ** 2
        v_wash = 2.0 * V_INDUCED * vertical_decay * radial_decay

        # --- base downward drag force on Drone A ---
        f_base = 0.5 * AIR_DENSITY * v_wash ** 2 * CD * A_EFF

        # --- turbulence overlay (band-limited noise) ---
        nx = self.noise_x.step()
        ny = self.noise_y.step()
        nz = self.noise_z.step()

        fz = -(f_base + f_base * TURB_INTENSITY * nz)
        fx = f_base * TURB_INTENSITY * TURB_LATERAL_SCALE * nx
        fy = f_base * TURB_INTENSITY * TURB_LATERAL_SCALE * ny

        # slight outward radial push from the wake
        if r_horiz > 0.01:
            radial_push = 0.15 * f_base * (r_horiz / sigma)
            fx += radial_push * (dx / r_horiz)
            fy += radial_push * (dy / r_horiz)

        return fx, fy, fz

    # -----------------------------------------------------------------
    def _apply_wrench(self, fx, fy, fz):
        wrench = Wrench()
        wrench.force = Vector3(fx, fy, fz)
        wrench.torque = Vector3(0, 0, 0)

        try:
            self.wrench_srv(
                body_name='iris::base_link',
                reference_frame='world',
                reference_point=Point(0, 0, 0),
                wrench=wrench,
                start_time=rospy.Time(0),
                duration=rospy.Duration(WRENCH_DURATION_S))
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f'apply_body_wrench failed: {e}')

    # -----------------------------------------------------------------
    def run(self):
        while not rospy.is_shutdown():
            fx, fy, fz = self._compute_force()

            if abs(fx) > 1e-6 or abs(fy) > 1e-6 or abs(fz) > 1e-6:
                self._apply_wrench(fx, fy, fz)

            msg = Vector3Stamped()
            msg.header = Header(stamp=rospy.Time.now(), frame_id='world')
            msg.vector = Vector3(fx, fy, fz)
            self.debug_pub.publish(msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        DownwashForce().run()
    except rospy.ROSInterruptException:
        pass
