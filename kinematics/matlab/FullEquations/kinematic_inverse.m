function [w1, w2, w3, w4, q1, q2, q3, q4] = kinematic_inverse(ky, km, Fx, Fy, Fz, Tx, Ty, Tz)

    syms w1 w2 w3 w4 q1 q2 q3 q4

    % Run the kinematic equations
    run("kinematic_equations.m")
    
    eq_Fx = fx1 + fx2 + fx3 + fx4 - Fx ;
    eq_Fy = fy1 + fy2 + fy3 + fy4 - Fy ;
    eq_Fz = fz1 + fz2 + fz3 + fz4 - Fz ;
    
%     eq_Tx = (tx1 + tx2 + tx3 + tx4) + ((fz1 - fz2) + (fy1 - fy2)) - Tx ;
%     eq_Ty = (ty1 + ty2 + ty3 + ty4) + ((fz3 - fz4) + (fx3 - fx4)) - Ty ;
%     eq_Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4)) - Tz ;
    eq_Tx = (tx1 + tx2 + tx3 + tx4) + (fz1 - fz2) - Tx ;
    eq_Ty = (ty1 + ty2 + ty3 + ty4) + (fz3 - fz4) - Ty ;
    eq_Tz = (tz1 + tz2 + tz3 + tz4) + ((fx1 - fx2) + (fy3 - fy4)) - Tz ;
    
    
    Equations       = [eq_Fx, eq_Fy, eq_Fz, eq_Tx, eq_Ty, eq_Tz] ; 
    Variables       = [w1, w2, w3, w4, q1, q2, q3, q4] ; 
    SearchIntervals = [0 200;
                       0 200;
                       0 200;
                       0 200;
                       -2* pi 2 *pi;
                       -2* pi 2 *pi;
                       -2* pi 2 *pi;
                       -2* pi 2 *pi] ;

    [w1, w2, w3, w4, q1, q2, q3, q4] = vpasolve(Equations, Variables, SearchIntervals, 'Random', false) ;

end

