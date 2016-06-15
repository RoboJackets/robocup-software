
function rad = DegreesToRadians(deg)
rad = deg * pi / 180;
end

WheelDist = 0.0798576;
WheelRadius = 0.02856;
WheelAngles = [
    DegreesToRadians(38),
    DegreesToRadians(315),
    DegreesToRadians(225),
    DegreesToRadians(142)
];

% See this paper for more info on how this matrix is derived:
% http://people.idsia.ch/~foerster/2006/1/omnidrive_kiart_preprint.pdf
BotToWheel = [
    sin(WheelAngles(1)), cos(WheelAngles(1)), -WheelDist;
    sin(WheelAngles(2)), cos(WheelAngles(2)), -WheelDist;
    sin(WheelAngles(3)), cos(WheelAngles(3)), -WheelDist;
    sin(WheelAngles(4)), cos(WheelAngles(4)), -WheelDist;
] / WheelRadius;

WheelToBot = pinv(BotToWheel);

% print matrices
WheelToBot
BotToWheel

% Example
targetVel = [0, 2, 0]'
targetWheels = BotToWheel * targetVel
