
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
    
    
    methods (Access = protected)

        % indicate that the output doesn't depend on the input directly,
        % only on the current state
        function [flag1, flag2, flag3, flag4, flag5] = isInputDirectFeedthroughImpl(~, ~, ~, ~, ~, ~)
            flag1 = false;
            flag2 = false;
            flag3 = false;
            flag4 = false;
            flag5 = false;
        end
        
        
        function resetImpl(obj)
            obj.X_g = [0 0 0]';
            obj.X_b_dot = [0 0 0]';
        end
        
        % Specify the number of inputs to the step() method.
        % inputs: u, dt
        function num = getNumInputsImpl(~)
            num = 5;
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
        
        
        function [flag1, flag2] = isOutputComplexImpl(obj)
            flag1 = false;
            flag2 = false;
        end
        
        
        function [X_g, X_b_dot] = outputImpl(obj, ~, ~, ~, ~, ~)
           X_g = obj.X_g;
           X_b_dot = obj.X_b_dot;
        end
        
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(obj, name)
            sz = [3 1];
            dt = 'double';
            cp = false; % not complex
        end
        
        
        function validateInputsImpl(~, u, dt, A_1, A_2, B)
            sz = size(u);
            if ~(sz(1) == 4 && sz(2) == 1)
               error('Wrong dimensions for input, should be [4, 1]') 
            end
            
            if dt < 0
                error('dt must be positive')
            end
        end
        
        function updateImpl(obj, u, dt, A_1, A_2, B)
            tspan = [0, dt];
            options = odeset;
            
            [t, xa] = ode45(@calculate_bot_accel, tspan, [obj.X_g; obj.X_b_dot], options, obj, u, A_1, A_2, B);
            
            result = xa(end, :);
            obj.X_g = result(1:3)';
            obj.X_b_dot = result(4:6)';
        end
    end
    
    
    
    % input and output names
    methods (Access = protected)
        function [in1name, in2name, in3name, in4name, in5name] = getInputNamesImpl(~)
            in1name = 'Motor Voltages';
            in2name = 'dt';
            in3name = 'A_1';
            in4name = 'A_2';
            in5name = 'B';
        end
        
        function [out1name, out2name] = getOutputNamesImpl(~)
            out1name = 'Robot pose';
            out2name = 'Robot velocity';
        end
    end
    
    
    % Custom name for the block in SimuLink
    methods (Access = protected)
        function icon = getIconImpl(~)
            icon = sprintf('RoboCup Robot\nDynamics');
        end
    end
    
end




% calculate X_b_dot_dot, the acceleration in the body frame
function accel = calculate_bot_accel(t, y, obj, u, A_1, A_2, B)
    X_g = y(1:3);
    X_b_dot = y(4:6);

    phi = X_g(3);
    gbR = [cos(phi), -sin(phi), 0;
           sin(phi),  cos(phi), 0;
                  0,         0, 1];

    dPhiDt = [0 0 1]*X_b_dot;

    X_b_dot_dot = A_1*X_b_dot + A_2*X_b_dot*dPhiDt + B*u;
    accel = [gbR*X_b_dot; X_b_dot_dot];
end
