#!/usr/bin/env python3

import numpy as np
from scipy.optimize import minimize
from math import sin, cos, pi

class KinematicsSolver:
    def __init__(self, ky=2.0, km=3.0):
        """
        Initialize the IK Solver.
        :param ky: Force constant (Force = ky * w)
        :param km: Moment constant (Torque = km * w)
        """
        self.ky = ky
        self.km = km
        
        # Initial guess: [w1, w2, w3, w4, q1, q2, q3, q4]
        # w = motor speed/throttle
        # q = motor angle (radians)
        self.last_solution = np.zeros(8) 
        
        # Define bounds
        # w: [0, 100] (Assuming normalized throttle 0-100 or similar, tune as needed)
        # q: [-pi, pi] (Full rotation)
        self.bounds = [(0, 100)]*4 + [(-pi, pi)]*4

    def forward_kinematics(self, params):
        """
        Calculate Forces (Fx, Fy, Fz) and Torques (Tx, Ty, Tz) given motor params.
        params: [w1, w2, w3, w4, q1, q2, q3, q4]
        """
        # Unpack parameters
        w1, w2, w3, w4 = params[0:4]
        q1, q2, q3, q4 = params[4:8]
        
        ky = self.ky
        km = self.km
        
        # Calculate individual Forces (f) and Torques (t) magnitude per motor
        # Note: Directions defined in kinematic_equations.m
        f1 = ky * w1
        f2 = ky * w2
        f3 = ky * w3
        f4 = ky * w4
        
        t1 = km * w1
        t2 = -km * w2
        t3 = km * w3
        t4 = -km * w4
        
        # --- Motor 1 (Contribution to X, Z) ---
        fx1 = -f1 * sin(q1)
        fy1 = 0
        fz1 = f1 * cos(q1)
        
        tx1 = -t1 * sin(q1)
        ty1 = 0
        tz1 = t1 * cos(q1)
        
        # --- Motor 2 (Contribution to X, Z) ---
        fx2 = -f2 * sin(q2)
        fy2 = 0
        fz2 = f2 * cos(q2)
        
        tx2 = -t2 * sin(q2)
        ty2 = 0
        tz2 = t2 * cos(q2)
        
        # --- Motor 3 (Contribution to Y, Z) ---
        fx3 = 0
        fy3 = -f3 * sin(q3)
        fz3 = f3 * cos(q3)
        
        tx3 = 0
        ty3 = -t3 * sin(q3)
        tz3 = t3 * cos(q3)
        
        # --- Motor 4 (Contribution to Y, Z) ---
        fx4 = 0
        fy4 = -f4 * sin(q4)
        fz4 = f4 * cos(q4)
        
        tx4 = 0
        ty4 = -t4 * sin(q4)
        tz4 = t4 * cos(q4)
        
        # --- Summation ---
        Fx = fx1 + fx2 + fx3 + fx4
        Fy = fy1 + fy2 + fy3 + fy4
        Fz = fz1 + fz2 + fz3 + fz4
        
        # Torque equations from kinematic_forward.m
        Tx = (tx1 + tx2 + tx3 + tx4) + (fz1 - fz2)
        Ty = (ty1 + ty2 + ty3 + ty4) + (fz3 - fz4)
        Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4))
        
        return np.array([Fx, Fy, Fz, Tx, Ty, Tz])

    def solve(self, target_forces, target_torques=[0,0,0]):
        """
        Solve Inverse Kinematics.
        :param target_forces: [Fx, Fy, Fz]
        :param target_torques: [Tx, Ty, Tz] (optional, default 0)
        :return: [w1, w2, w3, w4, q1, q2, q3, q4]
        """
        target = np.array(list(target_forces) + list(target_torques))
        
        def objective(params):
            current = self.forward_kinematics(params)
            
            # Weighted error
            # Prioritize Forces (especially for haptic feedback)
            error = current - target
            weights = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]) # Less weight on Torque
            weighted_error = error * weights
            
            # Regularization: Minimize energy (w) to prevent high speeds for no reason
            # Also helps solver stability
            reg_w = 0.01 * np.sum(params[0:4]**2)
            
            return np.sum(weighted_error**2) + reg_w

        # Use SLSQP (Sequential Least SQuares Programming) which handles bounds well
        res = minimize(
            objective, 
            self.last_solution, 
            bounds=self.bounds, 
            method='SLSQP',
            options={'ftol': 1e-4, 'disp': False, 'maxiter': 50} 
        )
        
        # Update last solution for warm start next time
        self.last_solution = res.x
        
        return res.x

