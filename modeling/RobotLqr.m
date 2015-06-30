
% Linear Quadratic Tracking Regulator controller for the robot
classdef RobotLqr < matlab.System & matlab.system.mixin.Propagates

    % These values determine Q and R for the LQR controller
    properties
        % Translational velocity error weight
        trans_vel_weight = 1;

        % Rotational velocity error weight
        % Be careful not to set this too high or the system will become
        % unstable and theta will oscillate ridiculously
        rot_vel_weight = 0.1;

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

            dPhiDt = currVel(3);
            % linearize by evaluating at current state
            A = A_1 + A_2*dPhiDt;

            % y = Cx + Du
            C = eye(3);
            D = zeros(3, 4);    % no feed-forward


            % We use LQR to calculate K, the optimal gain matrix, given our
            % plant model and weighting matrices Q and R
            [K, S, E] = lqr(A, B, obj.Q, obj.R);

            % Standard LQR is used to bring the system to rest (X = 0, u = 0).
            % However, in our case, we want to bring the robot to a
            % particular command velocity (a 'reference' in controls
            % speak).  To do this, instead of setting u = -K*X, we take
            % into account what control u would need to be applied when the
            % system is settled at the desired cmdVel.  When the system is
            % settled, there is no acceleration, so:
            % X_dot = A*cmdVel + B*u = 0
            % Solving for u, we get:
            % u = -pinv(B)*A*cmdVel
            % We add this value to -K*(currVel-cmdVel) to get our final
            % control values.
            u = -K*(currVel-cmdVel) - pinv(B)*A*cmdVel;

            % Go here and see the info on set-points:
            % http://www.academia.edu/6945404/Undergraduate_Lecture_Notes_on_LQG_LQR_controller_design

            % Nonlinear LQT based on this paper:
            % http://www.cs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf

            % Here's a good explanation of Lagrange multipliers:
            % http://www.slimy.com/~steuard/teaching/tutorials/Lagrange.html

            % Good (but long) intro to controls and LQR/LQT:
            % https://math.berkeley.edu/~evans/control.course.pdf
        end
    end

end
