clc
clear

% Define constants
ky = 2 ;
km = 3 ;

% Define w1 and q1 (deg)
w1_orig = randi([0 100]) ;
w2_orig = randi([0 100]) ;
q1_orig = deg2rad(randi([-180 180])) ;
q2_orig = deg2rad(randi([-180 180])) ;

% Start timer
tic

fprintf('\nInput values\n');
fprintf('w1 %f      w2 %f \n',w1_orig, w2_orig);
fprintf('q1 %f      q2 %f \n',q1_orig, q2_orig);

[Fx_Orig, Fy_Orig, Fz_Orig, Tx_Orig, Ty_Orig, Tz_Orig] = kinematic_forward(ky, km, w1_orig, w2_orig, q1_orig, q2_orig);

fprintf('\nIntermediate values\n');
fprintf('Fx %f      Fy %f      Fz %f\n',Fx_Orig, Fy_Orig, Fz_Orig);
fprintf('Tx %f      Ty %f      Tz %f\n',Tx_Orig, Ty_Orig, Tz_Orig);

[w1, w2, q1, q2] = kinematic_inverse(ky, km, Fx_Orig, Fy_Orig, Fz_Orig, Tx_Orig, Ty_Orig, Tz_Orig);

fprintf('\nOutput values\n');
fprintf('w1 %f      w2 %f      \n',w1, w2);
fprintf('q1 %f      q2 %f      \n',rad2deg(q1), rad2deg(q2));

if (isempty(w1))
    DeltaF = -1;
    fprintf('\nThese input did not work\n');
    fprintf('w1 %f      w2 %f\n',w1_orig, w2_orig);
    fprintf('q1 %f      q2 %f\n',q1_orig, q2_orig);
else

    [Fx, Fy, Fz, Tx, Ty, Tz] = kinematic_forward(ky, km, w1, w2, q1, q2);
   
    fprintf('\nFinal values\n');
    fprintf('Fx %f      Fy %f      Fz %f\n',Fx, Fy, Fz);
    fprintf('Tx %f      Ty %f      Tz %f\n',Tx, Ty, Tz);

    DeltaF = abs(Fx_Orig - Fx) + abs(Fy_Orig - Fy) + abs(Fz_Orig - Fz);
end

fprintf('\nDifference\n');
fprintf('Delta F: %f\n',DeltaF);

% End timer
toc



