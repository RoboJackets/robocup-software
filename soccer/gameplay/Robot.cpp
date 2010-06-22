#include "Robot.hpp"
#include "GameplayModule.hpp"

using namespace Geometry2d;

/** Constant for timestamp to seconds */
const float intTimeStampToFloat = 1000000.0f;

Gameplay::Robot::Robot(GameplayModule *gameplay, int id, bool self)
{
	_gameplay = gameplay;
	_id = id;
	_self = self;

	willKick = false;
	avoidBall = false;
	exclude = false;

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

void Gameplay::Robot::move(Geometry2d::Point pt, bool stopAtEnd)
{
	packet()->cmd.goalPosition = pt;

	// handle stop at end commands
	if (stopAtEnd)
		packet()->cmd.pathEnd = Packet::MotionCmd::StopAtEnd;
	else
		packet()->cmd.pathEnd = Packet::MotionCmd::FastAtEnd;

	// enable the RRT-based planner
	packet()->cmd.planner = Packet::MotionCmd::RRT;
}

void Gameplay::Robot::move(const std::vector<Geometry2d::Point>& path, bool stopAtEnd)
{
	// set motion command to use the explicit path generation
	packet()->cmd.planner = Packet::MotionCmd::Path;
	if (stopAtEnd)
		packet()->cmd.pathEnd = Packet::MotionCmd::StopAtEnd;
	else
		packet()->cmd.pathEnd = Packet::MotionCmd::FastAtEnd;

	// clear the path and set it to the correct one
	packet()->cmd.explicitPath.clear();
	packet()->cmd.explicitPath = path;
}

void Gameplay::Robot::bezierMove(const std::vector<Geometry2d::Point>& controls,
		Packet::MotionCmd::OrientationType facing,
		Packet::MotionCmd::PathEndType endpoint) {

	// set motion command to use the explicit path generation
	packet()->cmd.planner = Packet::MotionCmd::Bezier;
	packet()->cmd.pathEnd = endpoint;
	packet()->cmd.face = facing;

	// TODO: enable this - currently not used
//				// set the avoidance flag
//				packet()->cmd.enableBezierAvoid = enableAvoid;

	// set the control points
	packet()->cmd.bezierControlPoints.clear();
	packet()->cmd.bezierControlPoints = controls;
}

void Gameplay::Robot::move(const std::vector<Packet::MotionCmd::PathNode>& timedPath, uint64_t start) {
	// set controller type
	packet()->cmd.planner = Packet::MotionCmd::TimePosition;

	// set path
	packet()->cmd.timePosPath.clear();
	packet()->cmd.timePosPath = timedPath;

	// set start time
	packet()->cmd.start_time = start;
}

void Gameplay::Robot::directMotorCommands(const std::vector<int8_t>& speeds) {
	packet()->cmd.planner = Packet::MotionCmd::DirectMotor;
	packet()->cmd.direct_motor_cmds = speeds;
}

void Gameplay::Robot::directMotionCommands(const Geometry2d::Point& trans, double ang)
{
	packet()->cmd.planner = Packet::MotionCmd::DirectVelocity;
	packet()->cmd.direct_ang_vel = ang;
	packet()->cmd.direct_trans_vel = trans;
}

Packet::LogFrame::Robot * Gameplay::Robot::packet() const
{
	return _packet;
}

bool Gameplay::Robot::visible() const
{
	return packet()->valid;
}

int Gameplay::Robot::id() const
{
	return _id;
}

const Geometry2d::Point & Gameplay::Robot::pos() const
{
	return packet()->pos;
}

const Geometry2d::Point & Gameplay::Robot::vel() const
{
	return packet()->vel;
}

const float & Gameplay::Robot::angle() const
{
	return packet()->angle;
}

Geometry2d::Point Gameplay::Robot::pointInRobotSpace(const Geometry2d::Point& pt) const {
	Point p = pt;
	p.rotate(pos(), -1.0 * angle());
	return p;
}

const Geometry2d::Segment Gameplay::Robot::kickerBar() const {
	TransformMatrix pose(pos(), angle());
	const float mouthHalf = Constants::Robot::MouthWidth/2.0f;
	float x = sin(acos(mouthHalf/Constants::Robot::Radius))*Constants::Robot::Radius;
	Point L(x, Constants::Robot::MouthWidth/2.0f);
	Point R(x, -1.0 * Constants::Robot::MouthWidth/2.0f);
	return Segment(pose*L, pose*R);
}

bool Gameplay::Robot::behindBall(const Geometry2d::Point& ballPos) const {
	Point ballTransformed = pointInRobotSpace(ballPos);
	return ballTransformed.x < -Constants::Robot::Radius;
}


void Gameplay::Robot::setVScale(float scale) {
	packet()->cmd.vScale = scale;
}

float Gameplay::Robot::kickTimer() const {
	return (charged()) ? 0.0 : intTimeStampToFloat * (float) (_gameplay->state()->timestamp - _lastChargedTime);
}

void Gameplay::Robot::update() {
	if (charged())
		_lastChargedTime = _gameplay->state()->timestamp;
}


void Gameplay::Robot::spin(Packet::MotionCmd::SpinType dir)
{
	packet()->cmd.spin = dir;
}

bool Gameplay::Robot::haveBall() const
{
	// prevent ball sensor from reporting true when the ball is nowhere near
	Point ball = _gameplay->state()->ball.pos;
	float dist = pos().distTo(ball);
	return packet()->haveBall && dist < Constants::Robot::Radius + 0.1;
}

Packet::LogFrame::Robot::Rev Gameplay::Robot::rev() const
{
	switch (_packet->config.rev) {
	case ConfigFile::rev2008:
		return Packet::LogFrame::Robot::rev2008;
	case ConfigFile::rev2010:
		return Packet::LogFrame::Robot::rev2010;
	}
}

bool Gameplay::Robot::hasChipper() const {
	return _packet->config.rev == ConfigFile::rev2010;
}

void Gameplay::Robot::dribble(int8_t speed)
{
	packet()->radioTx.roller = speed;
}

void Gameplay::Robot::pivot(Geometry2d::Point ctr, Packet::MotionCmd::PivotType dir)
{
	packet()->cmd.pivotPoint = ctr;
	packet()->cmd.pivot = dir;
}

void Gameplay::Robot::face(Geometry2d::Point pt, bool continuous)
{
	packet()->cmd.goalOrientation = pt;
	packet()->cmd.face = continuous ? Packet::MotionCmd::Endpoint : Packet::MotionCmd::Continuous;
}

void Gameplay::Robot::faceNone()
{
	packet()->cmd.face = Packet::MotionCmd::None;
}

void Gameplay::Robot::kick(uint8_t strength)
{
	willKick = true;
	packet()->radioTx.kick = strength;
	packet()->radioTx.useChipper = false;
}

void Gameplay::Robot::chip(uint8_t strength)
{
	willKick = true;
	packet()->radioTx.kick = strength;
	packet()->radioTx.useChipper = true;
}

void Gameplay::Robot::pivot(Geometry2d::Point center, bool cw)
{
	packet()->cmd.pivotPoint = center;
	packet()->cmd.pivot = cw ? Packet::MotionCmd::CW : Packet::MotionCmd::CCW;
}

bool Gameplay::Robot::charged() const
{
	return packet()->radioRx.charged;
}

bool Gameplay::Robot::self() const
{
	return _self;
}

ObstacleGroup & Gameplay::Robot::obstacles() const
{
	return packet()->obstacles;
}

void Gameplay::Robot::approachOpp(Robot * opp, bool value) {
	int idx = opp->id();
	approachOpponent[idx] = value;
}
