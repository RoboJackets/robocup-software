% S-Function representing the robot's dynamics


% Resources:
% http://blogs.mathworks.com/seth/2012/06/25/creating-your-own-block-with-continuous-states-matlab-s-function/



function robot_sfcn(block)
    setup(block);
    params();
%endfunction



% TODO: port types
% double, Real



function setup(block)

    % inputs: column vector of motor voltages
    block.NumInputPorts = 1;
    
    % set input to sampling mode 'sample' as opposed to 'frame'
    block.InputPort(1).SamplingMode = 'Sample';
    block.OutputPort(1).SamplingMode = 'Sample';
    block.OutputPort(2).SamplingMode = 'Sample';
    
    % outputs: robot pose and body velocity
    block.NumOutputPorts = 2;

    block.NumDialogPrms  = 0;
    
    % Output depends on current state, not inputs
    block.InputPort(1).DirectFeedthrough = false;
%     block.InputPort(1).Dimensions = 1;
    block.InputPort(1).Dimensions = [4 1];
%     block.InputPort(1).Rows = 4;
    
    % column vectors with three entries
%     block.OutputPort(1).Dimensions = 3;
%     block.OutputPort(2).Dimensions = 3;
    block.OutputPort(1).Dimensions = [3 1];
    block.OutputPort(2).Dimensions = [3 1];
%     block.OutputPort(1).Size = 3;
%     block.OutputPort(2).Rows = 3;

    % continuous states: robot pose and body velocity
    block.NumContStates = 6;
    
    % Register methods
    block.RegBlockMethod('InitializeConditions',    @InitConditions);  
    block.RegBlockMethod('Outputs',                 @Output);
    block.RegBlockMethod('Derivatives',             @Derivatives);
    
%endfunction



function InitConditions(block)
    block.ContStates.Data(1:3) = [0 0 0]';    % initial pose is origin
    block.ContStates.Data(4:6) = [0 0 0]';    % zero initial velocity
%endfunction



function Output(block)
    block.OutputPort(1).Data = block.ContStates(1).Data;    % x_g
    block.OutputPort(2).Data = block.ContStates(2).Data;    % x_b_dot
%endfunction



function Derivatives(block)
    A_1 = - (K_e*K_t/R + K_f) / (M_bot*g^2*r^2 + I_asm) * eye(3);
    A_2 = - (M_bot*g^2*r^2)/(M_bot*g^2*r^2 + I_asm) * [0, 1, 0; -1, 0, 0; 0, 0, 0];
    B = g*r*K_t*G / R / (M_bot*g^2*r^2 + I_asm);

    block.Derivatives(2).Data = A_1*block.ContStates(1).Data + A_2*block.ContStates(1).Data * [0 0 1]*block.ContStates(1).Data + B*block.Inputs(1).Data;

    phi = [0 0 1]*block.Derivatives(1).Data;
    gbR = [cos(phi), -sin(phi), 0;
           sin(phi),  cos(phi), 0;
                  0,         0, 1];
    
    block.Derivatives(1).Data = gbR * block.Derivatives(2).Data;
%endfunction


