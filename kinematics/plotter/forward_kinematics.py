from math import sin
from math import cos

def forward_kinematics(ky, km, w, q):

    f1 = ky * w[0] 
    f2 = ky * w[1] 
    f3 = ky * w[2] 
    f4 = ky * w[3] 

    t1 = km * w[0] 
    t2 = -km * w[1] 
    t3 = km * w[2] 
    t4 = -km * w[3] 

    fx1 = -f1 * sin(q[0]) 
    fy1 = 0  
    fz1 = f1 * cos(q[0]) 
    tx1 = -t1 * sin(q[0]) 
    ty1 = 0 
    tz1 = t1 * cos(q[0]) 

    fx2 = -f2 * sin(q[1]) 
    fy2 = 0  
    fz2 = f2 * cos(q[1]) 
    tx2 = -t2 * sin(q[1]) 
    ty2 = 0 
    tz2 = t2 * cos(q[1]) 

    fx3 = 0 
    fy3 = -f3 * sin(q[2]) 
    fz3 = f3 * cos(q[2]) 
    tx3 = 0 
    ty3 = -t3 * sin(q[2]) 
    tz3 = t3 * cos(q[2]) 

    fx4 = 0 
    fy4 = -f4 * sin(q[3]) 
    fz4 = f4 * cos(q[3]) 
    tx4 = 0 
    ty4 = -t4 * sin(q[3]) 
    tz4 = t4 * cos(q[3]) 

    
    Fx = fx1 + fx2 + fx3 + fx4 
    Fy = fy1 + fy2 + fy3 + fy4 
    Fz = fz1 + fz2 + fz3 + fz4 

    Tx = (tx1 + tx2 + tx3 + tx4) + (fz1 - fz2) 
    Ty = (ty1 + ty2 + ty3 + ty4) + (fz3 - fz4) 
    Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4))

    return Fx, Fy, Fz, Tx, Ty, Tz, fx1, fx2, fx3, fx4, fy1, fy2, fy3, fy4, fz1, fz2, fz3, fz4

