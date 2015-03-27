
function [t, xa] = test_ode()

% odeset('OutputFcn', @()f(12,21))


u = [1, -1, -1, 1]';

x_b_dot_0 = [0 0 0]';
tspan = [0, 1/60];
[t, xa] = ode45(@f, tspan, x_b_dot_0, odeset, u)


function dydt = f(t, y, u)
params();

u
X_b_dot = y;

A_1 = - (K_e*K_t/R + K_f) / (M_bot*g^2*r^2 + I_asm) * eye(3);
A_2 = - (M_bot*g^2*r^2)/(M_bot*g^2*r^2 + I_asm) * [0, 1, 0; -1, 0, 0; 0, 0, 0];
B = g*r*K_t*G / R / (M_bot*g^2*r^2 + I_asm);

dydt = A_1*X_b_dot + A_2*X_b_dot * ([0 0 1]*X_b_dot) + B*u;
dydt = [dydt];
%endfunction
