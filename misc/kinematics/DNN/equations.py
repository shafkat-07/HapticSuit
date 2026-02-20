from math import sin, cos

def foward_kinematics(kw, theta_1, theta_2, theta_3, theta_4, omega_1, omega_2, omega_3, omega_4):
    # Compute the force of each motor
    f1 = kw * omega_1
    f2 = kw * omega_2
    f3 = kw * omega_3
    f4 = kw * omega_4

    # Compute the force in the x direction from each motor
    Fx1 = -(f1) * sin(theta_1)
    Fx2 = -(f2) * sin(theta_2)
    Fx3 = 0
    Fx4 = 0

    # Compute the force in the y direction from each motor
    Fy1 = 0
    Fy2 = 0
    Fy3 = -(f3) * sin(theta_3)
    Fy4 = (f4) * cos(theta_4)

    # Compute the force in the z direction from each motor
    Fz1 = (f1) * cos(theta_1)
    Fz2 = (f2) * cos(theta_2)
    Fz3 = (f3) * cos(theta_3)
    Fz4 = (f4) * cos(theta_4)

    # Compute the force in the resultant force
    result_Fx = Fx1 + Fx2 + Fx3 + Fx4
    result_Fy = Fy1 + Fy2 + Fy3 + Fy4
    result_Fz = Fz1 + Fz2 + Fz3 + Fz4

    return result_Fx, result_Fy, result_Fz