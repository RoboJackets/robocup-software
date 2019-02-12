% This script defines the robot parameters as symbols, then derives the
% A_1, A_2, and B matrices in terms of them.  This continues from the
% derivation I did on paper - I made it fairly far with that, but switched
% to MATLAB for the heavy symbol crunching.  The matrices derived here got
% copied into RobotParams.m as derived outputs.  Eventually, I'll put the
% full derivation in here for reference.
% note: this script will take a while to run...
% - Justin


syms theta0 theta1 theta2 theta3
syms L R r g I_asm
syms phi
syms M_bot
syms I_bot
syms K_e K_t K_f


G = [-sin(theta0), -sin(theta1), -sin(theta2), -sin(theta3);
      cos(theta0),  cos(theta1),  cos(theta2),  cos(theta3);
              1/L,          1/L,          1/L,          1/L];

J = [M_bot,     0,     0;
         0, M_bot,     0;
         0,     0, I_bot];

gbR = [cos(phi), -sin(phi), 0;
       sin(phi),  cos(phi), 0;
              0,         0, 1];


syms dPhiDt
gbR_dot = diff(gbR, 'phi') * dPhiDt;




M = 1/(g*r) * inv(J) * gbR * G;
M = simplify(M)

N = M*I_asm*pinv(G) / (g*r) + gbR;
N = simplify(N)


A_1 = -pinv(N) * ( (M*pinv(G)*K_e*K_t / (g*r*R)) + (M*K_f*pinv(G) / (g*r)) );
A_1 = simplify(A_1)


A_2 = -pinv(N)*gbR_dot;
A_2 = simplify(A_2)


B = pinv(N)*M*K_t / R;
B = simplify(B)

