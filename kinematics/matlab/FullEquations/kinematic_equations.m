f1 = ky * w1 ;
f2 = ky * w2 ;
f3 = ky * w3 ;
f4 = ky * w4 ;

t1 = km * w1 ;
t2 = -km * w2 ;
t3 = km * w3 ;
t4 = -km * w4 ;

fx1 = -f1 * sin(q1) ;
fy1 = 0  ;
fz1 = f1 * cos(q1) ;
tx1 = -t1 * sin(q1) ;
ty1 = 0 ;
tz1 = t1 * cos(q1) ;

fx2 = -f2 * sin(q2) ;
fy2 = 0  ;
fz2 = f2 * cos(q2) ;
tx2 = -t2 * sin(q2) ;
ty2 = 0 ;
tz2 = t2 * cos(q2) ;

fx3 = 0 ;
fy3 = -f3 * sin(q3) ;
fz3 = f3 * cos(q3) ;
tx3 = 0 ;
ty3 = -t3 * sin(q3) ;
tz3 = t3 * cos(q3) ;

fx4 = 0 ;
fy4 = -f4 * sin(q4) ;
fz4 = f4 * cos(q4) ;
tx4 = 0 ;
ty4 = -t4 * sin(q4) ;
tz4 = t4 * cos(q4) ;
