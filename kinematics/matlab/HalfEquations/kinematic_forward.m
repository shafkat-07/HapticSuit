

function [Fx, Fy, Fz, Tx, Ty, Tz] = kinematic_forward(ky, km, w1, w2, q1, q2)

    % Run the kinematic equations
    run("kinematic_equations.m")
    
    Fx = fx1 + fx2 ;
    Fy = fy1 + fy2 ;
    Fz = fz1 + fz2 ;

    Tx = (tx1 + tx2) + (fz1 - fz2) ;
    Ty = (ty1 + ty2) ;
    Tz = (tz1 + tz2) + (fx1 - fx2) ;

end



