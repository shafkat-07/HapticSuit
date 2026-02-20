function [Fx, Fy, Fz, Tx, Ty, Tz] = kinematic_forward(ky, km, w1, w2, w3, w4, q1, q2, q3, q4)

    % Run the kinematic equations
    run("kinematic_equations.m")
    
    Fx = fx1 + fx2 + fx3 + fx4 ;
    Fy = fy1 + fy2 + fy3 + fy4 ;
    Fz = fz1 + fz2 + fz3 + fz4 ;

    Tx = (tx1 + tx2 + tx3 + tx4) + (fz1 - fz2) ;
    Ty = (ty1 + ty2 + ty3 + ty4) + (fz3 - fz4) ;
    Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4));

end