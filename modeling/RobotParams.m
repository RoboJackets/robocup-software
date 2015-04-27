% Contains robot hardware parameters used in the model
classdef RobotParams < matlab.System

    % Note: The default values for the below properties are garbage - if
    % you want to set them, double-click on the RobotParams block in the
    % Simulink block-diagram model file.
    properties
        % Mass of bot (Kg)
        M_bot = 1;

        % Moment of inertia of bot about the z (vertical) axis through the center of mass (Kg*m^2)
        I_bot = 1;

        % Gear ratio of motor to wheel.  w_motor * g = w_wheel
        g = 2.0 / 7.0;

        % Radius of omni-wheels (meters)
        r = 0.03;

        % Distance from the center of wheel to the center of the robot (meters)
        L = 0.1;

        % Resistance of motor from terminal to terminal (ohms)
        Rt = 1;

        % Back-emf constant of motor (V/(rad/s))
        % Maxon lists the "Speed Constant" in rpm/V as 380
        % invert this to get 1/380 V/rpm
        % multiply by 60 to get 60/380 V/rps
        % divide by 2pi to get 30/380pi V/(rad/s)
        K_e = 30.0/(380*pi);

        % Torque constant of motor (N*m/A)
        K_t = 1;

        % Viscous friction coefficient of wheel assembly (N*m/(rad/s)).  Tau_friction = w_motor * K_f
        K_f = 1;

        % Moment of inertia of wheel assembly.  Tau_accel = I_asm*w_dot_motor
        I_asm = 1;

        % Max voltage applied to the motor phases (Volts)
        V = 1;

        % Angle of each wheel axis relative to the +x axis (radians)
        wheel_angles = (pi/4 + (pi/2)*(0:3))';
    end


    properties (Dependent = true)
        % Inertial matrix
        J;

        % Geometry matrix - maps wheel velocities to body velocity
        G;

        % matrices for final nonlinear state equation
        % x_b_dot_dot = A_1*x_b_dot + A_2*dphidt*x_b_dot + B*u
        A_1;
        A_2;
        B;
    end


    methods
        function J = get.J(obj)
            J = [obj.M_bot,         0,         0;
                         0, obj.M_bot,         0;
                         0,         0, obj.I_bot];
        end

        function G = get.G(obj)
            G = [-sin(obj.wheel_angles(1)), -sin(obj.wheel_angles(2)), -sin(obj.wheel_angles(3)), -sin(obj.wheel_angles(4));
                  cos(obj.wheel_angles(1)),  cos(obj.wheel_angles(2)),  cos(obj.wheel_angles(3)),  cos(obj.wheel_angles(4));
                                   1/obj.L,                   1/obj.L,                   1/obj.L,                   1/obj.L];
        end

        function A_1 = get.A_1(obj)
            A_1 = - (obj.K_e*obj.K_t/obj.Rt + obj.K_f) / (obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm) * eye(3);
        end

        function A_2 = get.A_2(obj)
            A_2 = (obj.M_bot*obj.g^2*obj.r^2)/(obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm) * [0, 1, 0; -1, 0, 0; 0, 0, 0];
        end
        
        function B = get.B(obj)
            B = obj.g*obj.r*obj.K_t*obj.G / obj.Rt / (obj.M_bot*obj.g^2*obj.r^2 + obj.I_asm);
        end
    end


    methods (Access = protected)
        function num = getNumOutputsImpl(~)
            num = 3;
        end
        
        function num = getNumInputsImpl(~)
            num = 0;
        end
        
        function [A_1, A_2, B] = stepImpl(obj)
           A_1 = obj.A_1;
           A_2 = obj.A_2;
           B = obj.B;
        end
        
        function [out1name, out2name, out3name] = getOutputNamesImpl(~)
            out1name = 'A_1';
            out2name = 'A_2';
            out3name = 'B';
        end
        
        function [sz_A_1, sz_A_2, sz_B] = getOutputSizeImpl(~)
            sz_A_1 = [3 3];
            sz_A_2 = [3 3];
            sz_B = [3 4];
        end
        
        function fixedout = isOutputFixedSizeImpl(~)
            fixedout = true;
        end
    end

end
