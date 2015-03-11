
% This class models the dynamics of a single RoboCup SSL Robot
% note: a good example of custom System blocks can be found at:
% http://www.mathworks.com/help/simulink/ug/system-design-in-simulink-using-system-objects.html
classdef Robot < matlab.System & matlab.system.mixin.CustomIcon
    
    
    % The state of the robot is a column vector containing the angular
    % velocity of each of the four motors
    properties (ContinuousState)
        X
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
        wheel_angles = (pi/4 + (pi/2)*0:3)';
    end
    
    
    % input and output names
    methods (Access = protected)
        function [in1name] = getInputNamesImpl(~)
            in1name = 'Motor Voltages';
        end
        
        function [out1name] = getOutputNamesImpl(~)
            out1name = 'Motor Velocities';
        end
    end
    
    
    % Custom name for the block in SimuLink
    methods (Access = protected)
        function icon = getIconImpl(~)
            icon = sprintf('RoboCup Robot\nDynamics')
        end
    end
    
end
