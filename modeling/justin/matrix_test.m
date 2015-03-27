


syms I_asm g r R


syms M_bot I_bot
J = [M_bot 0 0;
     0 M_bot 0;
     0 0 I_bot];

syms phi
gbR = [cos(phi), -sin(phi), 0;
       sin(phi), cos(phi), 0;
       0, 0, 1];

syms dPhiDt
gbR_dot = dPhiDt * [sin(phi), cos(phi), 0;
                   -cos(phi), sin(phi), 0;
                            0,       0, 0];

N = I_asm/(g^2 * r*2) * inv(J) * gbR + gbR;
P = I_asm/(g^2 * r^2) * inv(J) + eye(3);
% N = P*gbR;



% inv(gbR)*inv(P)*inv(J)*gbR
% >> 
% [ (g^2*r^2*cos(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)) + (g^2*r^2*sin(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)),                                                                                                                                                   0,                                 0]
% [                                                                                                                                                   0, (g^2*r^2*cos(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)) + (g^2*r^2*sin(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)),                                 0]
% [                                                                                                                                                   0,                                                                                                                                                   0, (g^2*r^2)/(I_bot*g^2*r^2 + I_asm)]
 
% recognize that cos(x)^2 + sin(x)^2 = 1
% >>
% [ (g^2*r^2*cos(phi)^2)/(M_bot*g^2*r^2 + I_asm) + (g^2*r^2*sin(phi)^2)/(M_bot*g^2*r^2 + I_asm),                                                                                           0,                                 0]
% [                                                                                           0, (g^2*r^2*cos(phi)^2)/(M_bot*g^2*r^2 + I_asm) + (g^2*r^2*sin(phi)^2)/(M_bot*g^2*r^2 + I_asm),                                 0]
% [                                                                                           0,                                                                                           0, (g^2*r^2)/(I_bot*g^2*r^2 + I_asm)]

% next, factor out (g^2*r^2)/(M_bot*g^2*r^2 + I_asm)
% [ (cos(phi)^2) + (sin(phi)^2), 0,                              0]
% [                              0, (cos(phi)^2) + (sin(phi)^2), 0]
% [                              0,                           0, 1]
% * (g^2*r^2)/(M_bot*g^2*r^2 + I_asm)

% use our trig identities again:
% (g^2*r^2)/(M_bot*g^2*r^2 + I_asm) * eye(3)

% Now we know that: inv(gbR)*inv(P)*inv(J)*gbR = (g^2*r^2)/(M_bot*g^2*r^2 + I_asm) * eye(3)










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Next, let's reduce this:
% 
% inv(gbR)*inv(P)*gbR_dot
% >>
% [                                                                                                                                                                               0, (M_bot*dPhiDt*g^2*r^2*cos(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)) + (M_bot*dPhiDt*g^2*r^2*sin(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)), 0]
% [ - (M_bot*dPhiDt*g^2*r^2*cos(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)) - (M_bot*dPhiDt*g^2*r^2*sin(phi)^2)/((cos(phi)^2 + sin(phi)^2)*(M_bot*g^2*r^2 + I_asm)),                                                                                                                                                                             0, 0]
% [                                                                                                                                                                               0,                                                                                                                                                                             0, 0]
 

% apply trig identity: sin(x)^2 + cos(x)^2 = 1
% [                                                                                                                       0, (M_bot*dPhiDt*g^2*r^2*cos(phi)^2)/(M_bot*g^2*r^2 + I_asm) + (M_bot*dPhiDt*g^2*r^2*sin(phi)^2)/(M_bot*g^2*r^2 + I_asm), 0]
% [ - (M_bot*dPhiDt*g^2*r^2*cos(phi)^2)/(M_bot*g^2*r^2 + I_asm) - (M_bot*dPhiDt*g^2*r^2*sin(phi)^2)/(M_bot*g^2*r^2 + I_asm),                                                                                                                     0, 0]
% [                                                                                                                       0,                                                                                                                     0, 0]
 

% factor out M_bot*dPhiDt*g^2*r^2 / (M_bot*g^2*r^2 + I_asm)
% [                             0, (cos(phi)^2) + (sin(phi)^2), 0]
% [ - (cos(phi)^2) - (sin(phi)^2),                           0, 0]
% [                             0,                           0, 0]
% * M_bot*dPhiDt*g^2*r^2 / (M_bot*g^2*r^2 + I_asm)


% trig identities
% [  0, 1, 0]
% [ -1, 0, 0]
% [  0, 0, 0]
% * dPhiDt * ( M_bot*g^2*r^2 / (M_bot*g^2*r^2 + I_asm) );



% ///////////////////////////////////////



