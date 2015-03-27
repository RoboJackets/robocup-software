
% Mass of bot (Kg)
M_bot = 1;

% Moment of inertia of bot about the z (vertical) axis throug the center of mass TODO: units
I_bot = 1;

% Gear ratio of motor to wheel.  w_motor * g = w_wheel
g = 2.0 / 7.0;

% Radius of omni-wheels (meters)
r = 0.03;

% Distance from the center of wheel to the center of the robot (meters)
L = 0.1;

% Resistance of motor phase (ohms)
R = 1;

% Back-emf constant of motor (TODO: units?)
K_e = 1;

% Torque constant of motor (TODO: units?)
K_t = 1;

% Viscous friction coefficient of wheel assembly.  Tau_friction =w_motor * K_f TODO: units?
K_f = 1;

% Moment of inertia of wheel assembly.  Tau_accel = I_asm*w_dot_motor
I_asm = 1;

% Max voltage applied to the motor phases (Volts)
V = 1;

% Angle of each wheel axis relative to the +x axis (radians)
wheel_angles = (pi/4 + (pi/2)*(0:3))';

% Inertial matrix
J = [M_bot,     0,     0;
         0, M_bot,     0;
         0,     0, I_bot];

% Geometry matrix
G = [-sin(wheel_angles(1)), -sin(wheel_angles(2)), -sin(wheel_angles(3)), -sin(wheel_angles(4));
      cos(wheel_angles(1)),  cos(wheel_angles(2)),  cos(wheel_angles(3)),  cos(wheel_angles(4));
                       1/L,                   1/L,                   1/L,                   1/L];

