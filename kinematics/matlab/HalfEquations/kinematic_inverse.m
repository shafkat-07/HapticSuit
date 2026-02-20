function [w1, w2, q1, q2] = kinematic_inverse(ky, km, Fx, Fy, Fz, Tx, Ty, Tz)

    syms w1 w2 q1 q2

    % Run the kinematic equations
    run("kinematic_equations.m")
    
    eq_Fx = fx1 + fx2 - Fx ;
    eq_Fy = fy1 + fy2 - Fy ;
    eq_Fz = fz1 + fz2 - Fz ;
    
    eq_Tx = (tx1 + tx2) + (fz1 - fz2) - Tx ;
    eq_Ty = (ty1 + ty2) - Ty ;
    eq_Tz = (tz1 + tz2) + (fx1 - fx2) - Tz ;
    
    Equations       = [eq_Fx, eq_Fy, eq_Fz, eq_Tx, eq_Ty, eq_Tz] ; 
    Variables       = [w1, w2, q1, q2] ; 
    SearchIntervals = [0 200;
                       0 200;
                       -2* pi 2 *pi;
                       -2* pi 2 *pi] ;

    [w1, w2, q1, q2] = vpasolve(Equations, Variables, SearchIntervals, 'Random', false) ;

end

