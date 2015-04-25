
% Linear Quadratic Tracking (LQT) controller for the robot
classdef RobotLqt < matlab.System & matlab.system.mixin.Propagates
    
    
    % These values determine Q and R for the LQR controller
    properties
        % Translational velocity error weight
        trans_vel_weight = 1;
        
        % Rotational velocity error weight
        rot_vel_weight = 1;
        
        % Control voltage weight
        u_weight = 1;
    end
    
    
    properties (Dependent = true)
        Q;
        R;
    end
    
    
    methods
        function Q = get.Q(obj)
            Q = [obj.trans_vel_weight,                    0,                  0;
                                    0, obj.trans_vel_weight,                  0;
                                    0                     0, obj.rot_vel_weight];
        end
       
        function R = get.R(obj)
            R = eye(4) * obj.u_weight;
        end
    end
    
    
    methods (Access = protected)

        % input: command body velocity
        function num = getNumInputsImpl(~)
            num = 5;
        end

        % output: voltages for motors
        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function sz = getOutputSizeImpl(~)
            sz = [4 1];
        end

        function t1 = getOutputDataTypeImpl(~)
            t1 = 'double';
        end

        function [in1name, in2name, in3name, in4name, in5name] = getInputNamesImpl(~)
            in1name = 'Cmd Velocity';
            in2name = 'Curr. Velocity';
            in3name = 'A_1';
            in4name = 'A_2';
            in5name = 'B';
        end

        function name = getOutputNamesImpl(~)
            name = 'u';
        end
        
        
        function fixedout = isOutputFixedSizeImpl(~)
            fixedout = true;
        end
        
        
        function flag1 = isOutputComplexImpl(~)
            flag1 = false;
        end

        
        % TODO: implement parameter validator
        

        function u = stepImpl(obj, cmdVel, currVel, A_1, A_2, B)
            
            % Plant Equations:
            % X = x_b_dot % state of the system is body velocity
            % X_dot = A_1*x_b_dot + A_2*dPhiDt*x_b_dot + B*u
            % y = x_b_dot = eye(3)*x_b_dot
            
            
            % linearize by evaluating at current angular velocity
            A = A_1 + A_2*currVel(3); % note: currVel(3) = dPhi/dt
            
            % y = Cx + Du
            C = eye(3);
            D = zeros(3, 4);    % no feed-forward
            
            % declare lqr as extrinsic, so the code generator doesn't try
            % to generate code for the innards of lqr
%             coder.extrinsic('lqr');
            
            [K, S, E] = lqr(-C*A, -C*B, obj.Q, obj.R);
            
            u = -K * (cmdVel-currVel);
            

            % Nonlinear LQT based on this paper:
            % http://www.cs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf
            
            % Here's a good explanation of Lagrange multipliers:
            % http://www.slimy.com/~steuard/teaching/tutorials/Lagrange.html
            
            % Good (but long) intro to controls and LQR/LQT:
            % https://math.berkeley.edu/~evans/control.course.pdf
        end
    end
    
end
