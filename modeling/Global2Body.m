% A simple block that converts from global velocity to body velocity.
classdef Global2Body < matlab.System

    methods (Access = protected)
        % Inputs = global velocity, phi (robot global angle)
        function num = getNumInputsImpl(~)
            num = 2;
        end

        % output: body velocity
        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function [in1name, in2name] = getInputNamesImpl(~)
            in1name = 'Global Velocity';
            in2name = 'Robot angle';
        end

        function [out1name] = getOutputNamesImpl(~)
            out1name = 'Body Velocity';
        end

        function body = stepImpl(obj, globalVel, phi)
            % rotation matrix that converts from body frame to globalVel frame
            gbR = [cos(phi), -sin(phi), 0;
                   sin(phi),  cos(phi), 0;
                          0,         0, 1];

            % Inversion of the usual equation: Global = gbR*body
            body = pinv(gbR)*globalVel;
        end
    end

end
