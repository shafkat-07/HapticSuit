clc
clear

% Define constants
ky = 2 ;
km = 3 ;

% Define w1 and q1 (deg)
w1_orig = 0 ;
w2_orig = 1 ;
w3_orig = 1.3 ;
w4_orig = 0.5 ;
q1_orig = deg2rad(-160) ;
q2_orig = deg2rad(0) ;
q3_orig = deg2rad(180) ;
q4_orig = deg2rad(20) ;

% Start timer
tic

fprintf('\nInput values\n');
fprintf('w1 %f      w2 %f      w3 %f       w4 %f\n',w1_orig, w2_orig, w3_orig, w4_orig);
fprintf('q1 %f      q2 %f      q3 %f       q4 %f\n',q1_orig, q2_orig, q3_orig, q4_orig);

[Fx_Orig, Fy_Orig, Fz_Orig, Tx_Orig, Ty_Orig, Tz_Orig] = kinematic_forward(ky, km, w1_orig, w2_orig, w3_orig, w4_orig, q1_orig, q2_orig, q3_orig, q4_orig);

fprintf('\nIntermediate values\n');
fprintf('Fx %f      Fy %f      Fz %f\n',Fx_Orig, Fy_Orig, Fz_Orig);
fprintf('Tx %f      Ty %f      Tz %f\n',Tx_Orig, Ty_Orig, Tz_Orig);

[w1, w2, w3, w4, q1, q2, q3, q4] = kinematic_inverse(ky, km, Fx_Orig, Fy_Orig, Fz_Orig, Tx_Orig, Ty_Orig, Tz_Orig);

fprintf('\nOutput values\n');
fprintf('w1 %f      w2 %f     w3 %f       w4 %f      \n',w1, w2, w3, w4);
fprintf('q1 %f      q2 %f     q3 %f       q4 %f      \n',rad2deg(q1), rad2deg(q2), rad2deg(q3), rad2deg(q4));

if (isempty(w1))
    DeltaF = -1;
    fprintf('\nThese input did not work\n');
    fprintf('w1 %f      w2 %f      w3 %f       w4 %f\n',w1_orig, w2_orig, w3_orig, w4_orig);
    fprintf('q1 %f      q2 %f      q3 %f       q4 %f\n',q1_orig, q2_orig, q3_orig, q4_orig);
else

    [Fx, Fy, Fz, Tx, Ty, Tz] = kinematic_forward(ky, km, w1, w2, w3, w4, q1, q2, q3, q4);
   
    fprintf('\nFinal values\n');
    fprintf('Fx %f      Fy %f      Fz %f\n',Fx, Fy, Fz);
    fprintf('Tx %f      Ty %f      Tz %f\n',Tx, Ty, Tz);

    DeltaF = abs(Fx_Orig - Fx) + abs(Fy_Orig - Fy) + abs(Fz_Orig - Fz);
end

fprintf('\nDifference\n');
fprintf('Delta F: %f\n',DeltaF);

% End timer
toc

fprintf('--------------------------------\n\n\n\n');



