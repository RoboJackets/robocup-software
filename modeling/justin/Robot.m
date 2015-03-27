
% This class models the dynamics of a single RoboCup SSL Robot
% note: a good example of custom System blocks can be found at:
% http://www.mathworks.com/help/simulink/ug/system-design-in-simulink-using-system-objects.html
% TODO: why nondirect?
% We inherit from the Propagates mixin because we need the ability to
% specify the sizes of the outputs.
classdef Robot < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.Nondirect & matlab.system.mixin.Propagates
    
    % The state of the robot is its global location and its body velocity
    properties (DiscreteState)
        X_g;
        X_b_dot;
    end
    
    
    properties
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
    end
    
    

    properties (Dependent = true)
        J % inertial matrix
        G % robot geometry matrix.  maps wheel velocities/forces to robot velocities/forces
        
        % matrices for final nonlinear state equation
        % x_dot = A_1*x + A_2*dphidt*x + B*u
        A_1
        A_2
        B
    end
    
    methods
        function J = get.J(obj)
            J = [obj.M_bot 0 0;
                0 obj.M_bot 0;
                0 0 obj.I_bot];
        end
        
        function G = get.G(obj)
            G = [-sin(obj.wheel_angles(1)), -sin(obj.wheel_angles(2)), -sin(obj.wheel_angles(3)), -sin(obj.wheel_angles(4));
                  cos(obj.wheel_angles(1)),  cos(obj.wheel_angles(2)),  cos(obj.wheel_angles(3)),  cos(obj.wheel_angles(4));
                                   1/obj.L,                   1/obj.L,                   1/obj.L,                   1/obj.L];
        end
        
        function A_1 = get.A_1(obj)
            A_1 = - (obj.K_e*obj.K_t/obj.R + obj.K_f) / (obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm) * eye(3);
        end
        
        function A_2 = get.A_2(obj)
            A_2 = - (obj.M_bot*obj.g^2*obj.r^2)/(obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm) * [0, 1, 0; -1, 0, 0; 0, 0, 0];
        end
        
        function B = get.B(obj)
            B = obj.g*obj.r*obj.K_t*obj.G / obj.R / (obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm);
        end
    end
    
    
    
    
    methods (Access = protected)

        % indicate that the output doesn't depend on the input directly,
        % only on the current state
        function [flag1, flag2] = isInputDirectFeedthroughImpl(~, ~, ~)
            flag1 = false;
            flag2 = false;
        end
        
        
        function resetImpl(obj)
            obj.X_g = [0 0 0]';
            obj.X_b_dot = [0 0 0]';
        end
        
        % Specify the number of inputs to the step() method.
        % inputs: u, dt
        function num = getNumInputsImpl(~)
            num = 2;
        end
        
        % outputs: X_g, X_b_dot
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        
        
        function [sz_X_g, sz_X_b_dot] = getOutputSizeImpl(~)
           sz_X_g = [3 1];
           sz_X_b_dot = [3 1];
        end
        
        
        function [flag1, flag2] = isOutputFixedSizeImpl(obj)
            flag1 = true;
            flag2 = true;
        end
        
        
        function [t1, t2] = getOutputDataTypeImpl(obj)
            t1 = 'double';
            t2 = 'double';
        end
        
        
        function [X_g, X_b_dot] = outputImpl(obj, ~, ~)
           X_g = obj.X_g;
           X_b_dot = obj.X_b_dot;
        end
        
        
        function [flag1, flag2] = isOutputComplexImpl(obj)
            flag1 = false;
            flag2 = false;
        end
        
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(obj, name)
            sz = [3 1];
            dt = 'double';
            cp = false; % not complex
        end
        
        
        function validateInputsImpl(~, u, dt)
            sz = size(u);
            if ~(sz(1) == 4 && sz(2) == 1)
               error('Wrong dimensions for input, should be [4, 1]') 
            end
            
            dt;
            
            if dt < 0
                error('dt must be positive')
            end
        end
        
        function updateImpl(obj, u, dt)
            tspan = [0, dt];
            options = odeset;
            
%             [t, xa] = ode45(@f, tspan, x_b_dot_0, odeset, u)
            
            [t, xa] = ode45(@calculate_bot_accel, tspan, [obj.X_g; obj.X_b_dot], options, obj, u);
            
            xa;
            
            dt;
            
% @(t,y)evalfcn(t,y,arg1,arg2,...)
            result = xa(end, :);
            obj.X_g = result(1:3)';
            obj.X_b_dot = result(4:6)';
            
            
            obj.X_b_dot
            obj.X_g;
        end
    end
    
    
    
    % input and output names
    methods (Access = protected)
        function [in1name, in2name] = getInputNamesImpl(~)
            in1name = 'Motor Voltages';
            in2name = 'dt';
        end
        
        function [out1name, out2name] = getOutputNamesImpl(~)
            out1name = 'Robot pose';
            out2name = 'Robot velocity';
        end
    end
    
    
    % Custom name for the block in SimuLink
    methods (Access = protected)
        function icon = getIconImpl(~)
%             image(imread('robot_render.jpg'))
            icon = sprintf('RoboCup Robot\nDynamics');
        end
    end
    
end




% calculate X_b_dot_dot, the acceleration in the body frame
function accel = calculate_bot_accel(t, y, obj, u)

    X_g = y(1:3);
    X_b_dot = y(4:6);

    phi = X_g(3);
    gbR = [cos(phi), -sin(phi), 0;
           sin(phi),  cos(phi), 0;
                  0,         0, 1];

    dPhiDt = [0 0 1]*X_b_dot;

    X_b_dot_dot = obj.A_1*X_b_dot + obj.A_2*X_b_dot*dPhiDt + obj.B*u;
    accel = [gbR*X_b_dot; X_b_dot_dot];
end
