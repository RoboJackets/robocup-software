
%% Robot Coordinate System
%      +y
%       ^
%       |
%       |
% w_1/-----\w_0
%   / mouth \
%  |         | ---> +x
%   \       /
% w_2\-----/w_3
%



%% Hard Limits (for safety)

limits.max_accel = 3;           % m/s^2
limits.max_speed = 3.5;         % m/s
limits.max_angle_accel = pi;    % rad/s^2
limits.max_angle_speed = 2*pi;  % rad/s
limits.max_motor_current = 2;   % amps


%% Motor specs
% See http://www.maxonmotorusa.com/maxon/view/product/motor/ecmotor/ecflat/ecflat45/339285

% Maxon Motors Useful Equations:
% http://www.electromate.com/db_support/attachments/Maxon%20Motor%20Useful%20Equations.pdf



%% Simplifying assumptions
% Because mechanical factors dominate over electrical ones, we leave
% motor current out of the system's state vector and for the purposes
% of the robot-wide controls, essentially assume motor inductance is zero.
% ECE 4550 Lab 6 does the same thing, so I don't feel bad about making this
% approximation.



%% Relating desired torque to motor inputs

% Load torque on the motor comes from the torque required to accelerate
% the robot.

% T_e = B*w_m + J* wheel_rot_acc + T_load;
% T_e = 2*emf_avg*current/w_m = B*w_m + wheel_asm.rotational_inertia*rot_acc + T_l
% current = w_m*(B*w_m + J*d(w_m)/dt + T_load)/(2*emf_avg)
%FIXME: rot_inertia*rot*acc doesn't work needs to account for gears, etc. and their ratios

%FIXME: verify that emf_avg is legit, same with current
%FIXME: plot these on graphs



%% LQR Controller
% The main idea of Linear Quadratic Regulator (LQR) Control is to choose a
% control input to apply that minimizes a cost function with respect to the
% dynamics equations of the plant.  We already have the plant described by
% the A, B, C, and D matrices, so the next step is to decide on a cost
% function, then solve for the control input to apply.
% The cost function is written in terms of the matrix Q and R, which
% describe the cost for a given state and control input respectively.  The
% cost function is of the form $J = \int_0^{t_f}\left(x^TQx+u^TRu\right)dt$.
% $x^TQx$ and $u^TRu$ should always be non-negative for any $u$ and $x$,
% which is called positive semi-definite.  To minimize $J$, we'll take the
% gradient of $J$ w.r.t. $u$ and set it to zero and solve.  The result is a
% matrix $K$, which is used as: $u = -Kx$.
%
%
% We want to find Q and R.  A few notes about the properties of these two
% matrices:
% * errors in vel_x and vel_y should be weighted the same
% * it's most important for us to reduce trajectory-tracking errors, but at
%   the same time, we'd also like to reduce input currents to the motor
%   drivers to save power and not exceed current limits.
% * we would also like to reduce rapid changes in control inputs as much as
%   much as possible
%
% Q is nxn where n is the number of state variables
% R is mxn where m is the number of control inputs

% calculate Q
trans_wt = 1 / (limits.max_speed^2);
rot_wt = 1 / (limits.max_angle_speed^2);
% Q = [trans_wt    0       0;
%         0     trans_wt   0;
%         0        0     rot_wt];

% calculate R
% current_wt = 1 / (limits.max_angle_speed^2);   % we weight each output current equally
% R = current_wt*eye(4);


% TODO: is our system linear when we have currents as our inputs to the system or do we need to linearize it to use LQR?



% FIXME: With LQR, how do we weight cost for error in x, rather than x itself?
% This paper talks about it: http://www.egr.msu.edu/classes/me851/jchoi/lecture/LQI-note.pdf

% TODO: read http://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf

%TODO: look into Bryson's Method, which is a heuristic for determining Q and R
%TODO: look into root locus
%TODO: look into pole placement for controller design

% Example LQR: http://ocw.mit.edu/courses/aeronautics-and-astronautics/16-30-feedback-control-systems-fall-2010/lecture-notes/MIT16_30F10_lec12.pdf



%% Discretization

% TODO



%% Resources
% http://www.ohio.edu/people/williar4/html/PDF/JRAS08.pdf
% http://www.sciencedirect.com/science/article/pii/S0921889011001230 - MRL Paper





%% Try 2 of robot dynamics



% inertia matrix of robot
J = [M_bot   0     0;
       0   M_bot   0;
       0     0   I_bot];

% wheel angles relative to +x axis
thetas = (pi/4)+(pi/2)*(0:4);

% robot geometry matrix.  maps wheel velocities to body velocity
G = [-sin(thetas(1)), -sin(thetas(2)), -sin(thetas(3)), -sin(thetas(4));
      cos(thetas(1)),  cos(thetas(2)),  cos(thetas(3)),  cos(thetas(4));
           1/L,            1/L,            1/L,            1/L];




% TODO: explain these later


% % control inputs - voltages to each motor
% u = [1 -1 -1 1]';
% 
% % state of robot - angular velocity of each motor
% x = [0 0 0 0]';



% % derivative of phi
% x_bot_dot = G*x;
% phi_dot = x_bot_dot(3);

% % derivative of gbR
% gbR_dot = [-sin(phi)*phi_dot, -cos(phi)*phi_dot 0
%             cos(phi)*phi_dot, -sin(phi)*phi_dot 0
%                     0                  0        0];

% A = - pinv(Q)*(gbR_dot*G*g*r + P*K_e*K_t/R - K_f);
% B = pinv(Q)*P*K_t/R;
% x_dot = A*x + B*u;

% bot_acc = G*x_dot;




%% robocup test


% Path
dt = 1.0 / 60.0;
duration = 5;
tt = 0:dt:duration;
acc = 2;
angAcc = 0.1;
x_bot_dot_expected = tt*acc;
x_bot_expected = 0.5*acc*tt.^2;


x = [0 0 0 0]';
state_history = [x];
x_g_dot_history = [0 0 0]';
x_g = [0 0 0]';
x_g_history = [x_g];
for i = tt
    % derivative of phi
    x_bot_dot = G*x*g*r;
    phi_dot = x_bot_dot(3);
    
    phi = x_g(3);

    % body to global coordinate rotation matrix
    gbR = [cos(phi) -sin(phi) 0
           sin(phi)  cos(phi) 0
              0         0     1];
    
    % M and N are just globs of variables so I don't have to repeat too much
    M = 1/(g*r)*inv(J)*gbR*G;
    N = M*I_asm + gbR*G*g*r;
    
    % derivative of gbR
    gbR_dot = [-sin(phi)*phi_dot, -cos(phi)*phi_dot 0
                cos(phi)*phi_dot, -sin(phi)*phi_dot 0
                        0                  0        0];

%     A = - pinv(Q)*(gbR_dot*G*g*r + P*K_e*K_t/R - K_f);
%     B = pinv(Q)*P*K_t/R;
    A = -pinv(N)*(M*K_e*K_t/R + M*K_f + gbR_dot*G*g*r);
    B = pinv(N)*M*K_t/R;
    
    % open-loop control
    x_g_dot_dot_desired = [0 acc angAcc]'; % CORRECT
    
%     x_b_dot_dot_desired = pinv(gbR)*(x_g_dot_dot_desired - gbR_dot*G*x*g*r);
%     
%     x_dot_desired = pinv(G)*x_b_dot_dot_desired / (g*r);
    
    
    
    
    x_dot_desired = pinv(gbR*G)*(x_g_dot_dot_desired - gbR_dot*G*x*g*r) / (g*r);
    u = pinv(B)*(x_dot_desired - A*x);
    x_dot = A*x + B*u;
    x = x + x_dot*dt;
    
    
    x_g_dot_dot = gbR_dot*g*r*G*x + gbR*g*r*G*x_dot;
    
    
    tau_ext = (K_t/R)*u - (K_e*K_t/R)*x - K_f*x - I_asm*x_dot;
    
    state_history = [state_history x];
    
    x_g_dot = gbR*G*x*g*r;
    x_g_dot_history = [x_g_dot_history x_g_dot];
    x_g = x_g + x_g_dot*dt;
    x_g_history = [x_g_history x_g];
end

% figure

body_vel_history = G*state_history;

% subplot(2,1,1)
% plot([tt tt(end)+dt], body_vel_history(2,:))
% title('body vel.y vs time')

% subplot(2,1,2)
% plot([tt tt(end)+dt], body_vel_history(3,:))
% title('body vel.theta vs time')

% % graph try 2
tt_extended = [tt tt(end)+dt];
% [hAx, hLine1, hLine2] = plotyy(tt_extended, body_vel_history(1,:), tt_extended, body_vel_history(2,:));
% title('X and Y body velocities of robot')
% xlabel('Time (seconds)')
% ylabel(hAx(1), 'BodyVel.x')
% ylabel(hAx(2), 'BodyVel.y')


plotyy(tt_extended, x_g_history(1,:), tt_extended, x_g_history(2,:))
xlabel('Time (seconds)')
ylabel('FieldPos.x (meters)')
zlabel('FieldPos.y (meters)')

