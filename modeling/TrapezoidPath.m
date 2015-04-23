
% Accelerates at constant accel for a fixed time, then goes constant
% velocity for a bit, then decelerates back down to zero velocity
function x_g = TrapezoidPath(t)
% if isvector(t)
%     error('Error: parameter t must be a scalar')
% end

accel = [0 2 0.4]';
rampTime = 2;
flatTime = 2;

if t < rampTime
    x_g = 0.5*accel*t^2;
    return
end

distSoFar = 0.5*accel*rampTime^2;
remainingTime = t - rampTime;

if t < rampTime + flatTime
    x_g = distSoFar + (rampTime*accel)*remainingTime;
    return
end

distSoFar = distSoFar + (rampTime*accel)*flatTime;
remainingTime = remainingTime - flatTime;

if t < rampTime + flatTime + rampTime
    x_g = distSoFar - 0.5*accel*remainingTime^2 + (rampTime*accel)*remainingTime;
    return
end

x_g = distSoFar + 0.5*accel*rampTime^2;
