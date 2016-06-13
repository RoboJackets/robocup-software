
function rad = DegreesToRadians(deg)
rad = deg * 2 * pi;
end

WheelDist = 0.0798576;
WheelRadius = 0.02856;
WheelAngles = [
    DegreesToRadians(38),
    DegreesToRadians(315),
    DegreesToRadians(225),
    DegreesToRadians(142)
];

pi2r = 2 * pi * WheelRadius;
WheelToBot = [
    -pi2r*sin(WheelAngles(1)), -pi2r*sin(WheelAngles(2)), -pi2r*sin(WheelAngles(3)), -pi2r*sin(WheelAngles(4));
    pi2r*cos(WheelAngles(1)), pi2r*cos(WheelAngles(2)), pi2r*cos(WheelAngles(3)), pi2r*cos(WheelAngles(4));
    WheelRadius/WheelDist/4, WheelRadius/WheelDist/4, WheelRadius/WheelDist/4, WheelRadius/WheelDist/4;
];

BotToWheel = pinv(WheelToBot);

% print results
WheelToBot
BotToWheel
