% plots a robot's trajectory vs its desired trajectory over time in a cool animation
function visualize(target_pos, actual_pos)

dt = 0.01; % amount of time between successive points

figure(1)
pbaspect([6,9,1])   % square aspect ratio for display
title('Robot Trajectory-Tracking Animation')
xlabel('FieldPos.x (meters)')
ylabel('FieldPos.y (meters)')
hold on

% draw black circle for robot
bot_radius = 0.09;
bot_loc = [1, 1]';
rect = [bot_loc(1)-bot_radius, bot_loc(2)-bot_radius, bot_radius*2, bot_radius*2];
robot = rectangle('Position', rect, 'Curvature', [1,1], 'FaceColor', 'k');

% draw desired trajectory
plot(target_pos(1,:), target_pos(2,:), 'Color', 'b')

[m, n] = size(target_pos);
for i = 1:n
    % update robot location
    % TODO: include robot orientation
    bot_loc = actual_pos(:,i);
    rect = [bot_loc(1)-bot_radius, bot_loc(2)-bot_radius, bot_radius*2, bot_radius*2];
    set(robot, 'Position', rect);
    
    % draw robot position history
    plot(actual_pos(1,1:i), actual_pos(2,1:i), 'Color', 'k')

    % Set to field dimensions
    % we do this at each iteration because MATLAB tries to adjust them to fit the data
    axis([-3 3 0 9])
    
    pause(dt);
end
