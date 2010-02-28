#include "Robot.hpp"
#include "GameplayModule.hpp"

Gameplay::Robot::Robot(GameplayModule *gameplay, int id, bool self)
{
	_gameplay = gameplay;
	_id = id;
	_self = self;

	willKick = false;
	avoidBall = false;

	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		approachOpponent[i] = false;
	}

	if (_self)
	{
		_packet = &_gameplay->state()->self[_id];
	} else {
		_packet = &_gameplay->state()->opp[_id];
	}
}

void Gameplay::Robot::resetMotionCommand()
{
	packet()->cmd = Packet::MotionCmd();

	// Stay in place if possible.
	move(pos());
}

void Gameplay::Robot::updatePoseHistory()
{
	Packet::LogFrame::Robot::Pose pose;
	pose.pos = pos();
	pose.angle = angle();

	if (!_poseHistory.empty() && pose.pos.nearPoint(_poseHistory.back().pos, 0.01f))
	{
		return;
	}

	for (unsigned int i = 1; i < _poseHistory.size(); ++i)
	{
		_poseHistory[i - 1] = _poseHistory[i];
	}

	if (_poseHistory.size() < 400)
	{
		_poseHistory.push_back(pose);
	} else {
		_poseHistory.back() = pose;
	}

	_packet->poseHistory = _poseHistory;
}
