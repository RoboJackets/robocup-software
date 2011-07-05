#include "Env.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/SimCommand.pb.h>

#include <sys/time.h>
#include <Constants.hpp>
#include <Network.hpp>
#include <boost/foreach.hpp>
#include <Geometry2d/util.h>

using namespace std;
using namespace Geometry2d;
using namespace Packet;

static const QHostAddress LocalAddress(QHostAddress::LocalHost);
static const QHostAddress MulticastAddress(SharedVisionAddress);

const int Oversample = 1;

Env::Env()
{
	sendShared = false;
	_stepCount = 0;
	_frameNumber = 0;
	_dropFrame = false;
	ballVisibility = 100;
	
	//initialize the PhysX SDK
	_physicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
	assert(_physicsSDK);

	NxReal myScale = 0.5f;
	_physicsSDK->setParameter(NX_VISUALIZATION_SCALE, myScale);
	//gPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES, 2.0f);
	_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_SKELETONS, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_AABBS, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_BODY_LIN_VELOCITY, 1.0f);

	_physicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1.0f);

	_physicsSDK->setParameter(NX_SKIN_WIDTH, 0.001f); //depth of the two skins
	_physicsSDK->setParameter(NX_CONTINUOUS_CD, true);
	_physicsSDK->setParameter(NX_CCD_EPSILON, .001f);

	//setup a new scene
	NxSceneDesc sceneDesc;
	sceneDesc.setToDefault();

	//setup gravity to world gravity
	sceneDesc.gravity.set(0, 0, -9.81);

	//set z-axis as up axis
	sceneDesc.upAxis = 2;

	//setup the bounds for the scene, use field bounds
	//TODO field bounds
	NxBounds3 bounds;
	bounds.set(-10, -10, 0, 10, 10, 10); //minx,y,z, maxx,y,z
	sceneDesc.maxBounds = &bounds;

	_scene = _physicsSDK->createScene(sceneDesc);
	assert(_scene);

	_field = new Field(this);

	// Bind sockets
	assert(_visionSocket.bind(SimCommandPort));
	assert(_radioSocket[0].bind(RadioTxPort));
	assert(_radioSocket[1].bind(RadioTxPort + 1));
	
	gettimeofday(&_lastStepTime, 0);
	
	connect(&_timer, SIGNAL(timeout()), SLOT(step()));
	_timer.start(16 / Oversample);
}

Env::~Env()
{
	if (_scene)
	{
		_physicsSDK->releaseScene(*_scene);
		_scene = 0;
	}

	_physicsSDK->release();
	_physicsSDK = 0;
}

NxDebugRenderable Env::dbgRenderable() const
{
	return *_scene->getDebugRenderable();
}

void Env::step()
{
	// Check for SimCommands
	while (_visionSocket.hasPendingDatagrams())
	{
		string buf;
		unsigned int n = _visionSocket.pendingDatagramSize();
		buf.resize(n);
		_visionSocket.readDatagram(&buf[0], n);
		
		SimCommand cmd;
		if (!cmd.ParseFromString(buf))
		{
			printf("Bad SimCommand of %d bytes\n", n);
			continue;
		}
		
		if (!_balls.empty())
		{
			if (cmd.has_ball_vel())
			{
				_balls[0]->velocity(cmd.ball_vel().x(), cmd.ball_vel().y());
			}
			if (cmd.has_ball_pos())
			{
				_balls[0]->position(cmd.ball_pos().x(), cmd.ball_pos().y());
			}
		}
		
		BOOST_FOREACH(const SimCommand::Robot &rcmd, cmd.robots())
		{
			const RobotMap &team = rcmd.blue_team() ? _blue : _yellow;
			RobotMap::const_iterator i = team.find(rcmd.shell());
			
			if (i == team.end())
			{
				if (rcmd.has_pos())
				{
					// add a new robot
					addRobot(rcmd.blue_team(), rcmd.shell(), rcmd.pos(), Robot::rev2008); // TODO: make this check robot revision
				}
				else
				{
					// if there's no position, we can't add a robot
					printf("Trying to override non-existent robot %d:%d\n", rcmd.blue_team(), rcmd.shell());
					continue;
				}
			}

			// remove a robot if it is marked not visible
			if (rcmd.has_visible() && !rcmd.visible()) {
				removeRobot(rcmd.blue_team(), rcmd.shell());
				continue;
			}
			
			// change existing robots
			Robot *robot = *i;
			
			if (rcmd.has_pos())
			{
				robot->position(rcmd.pos().x(), rcmd.pos().y());
			}
			
			float new_w = 0.0;
			if (rcmd.has_w())
			{
				new_w = rcmd.w();
				if (!rcmd.has_vel())
				{
					robot->velocity(0.0, 0.0, new_w);
				}
			}

			if (rcmd.has_vel())
			{
				robot->velocity(rcmd.vel().x(), rcmd.vel().y(), new_w);
			}

			if (cmd.has_reset() && cmd.reset()) {
				// TODO: reset the robots to their initial config
				printf("Resetting to initial config - NOT IMPLEMENTED");
			}
		}
	}
	
	// Check for RadioTx packets
	for (int r = 0; r < 2; ++r)
	{
		QUdpSocket &s = _radioSocket[r];
		while (s.hasPendingDatagrams())
		{
			string buf;
			unsigned int n = s.pendingDatagramSize();
			buf.resize(n);
			s.readDatagram(&buf[0], n);
			
			RadioTx tx;
			if (!tx.ParseFromString(buf))
			{
				printf("Bad RadioTx on %d of %d bytes\n", r, n);
				continue;
			}
			
			handleRadioTx(r, tx);
		}
	}
	
	// Run physics
	struct timeval tv;
	gettimeofday(&tv, 0);
	_scene->simulate(tv.tv_sec - _lastStepTime.tv_sec + (tv.tv_usec - _lastStepTime.tv_usec) * 1.0e-6);
	_lastStepTime = tv;
	_scene->flushStream();
	_scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

	// Send vision data
	++_stepCount;
	if (_stepCount == Oversample)
	{
		_stepCount = 0;
		
		if (_dropFrame)
		{
			_dropFrame = false;
		} else {
			sendVision();
		}
	}
}

void Env::sendVision()
{
	SSL_WrapperPacket wrapper;
	SSL_DetectionFrame *det = wrapper.mutable_detection();
	det->set_frame_number(_frameNumber++);
	det->set_camera_id(0);
	
	struct timeval tv;
	gettimeofday(&tv, 0);
	det->set_t_capture(tv.tv_sec + (double)tv.tv_usec * 1.0e-6);
	det->set_t_sent(det->t_capture());
	
	BOOST_FOREACH(Robot *robot, _yellow)
	{
		if ((rand() % 100) < robot->visibility)
		{
			SSL_DetectionRobot *out = det->add_robots_yellow();
			convert_robot(robot, out);
		}
	}
	
	BOOST_FOREACH(Robot *robot, _blue)
	{
		if ((rand() % 100) < robot->visibility)
		{
			SSL_DetectionRobot *out = det->add_robots_blue();
			convert_robot(robot, out);
		}
	}
	
	Geometry2d::Point cam0(-Field_Length / 4, 0);
	Geometry2d::Point cam1(Field_Length / 4, 0);

	BOOST_FOREACH(const Ball* b, _balls)
	{
		Geometry2d::Point ballPos = b->getPosition();

		bool occ;
		if (ballPos.x < 0)
		{
			occ = occluded(ballPos, cam0);
		} else {
			occ = occluded(ballPos, cam1);
		}

		if (!occ && (rand() % 100) < ballVisibility)
		{
			SSL_DetectionBall *out = det->add_balls();
			out->set_confidence(1);
			out->set_x(ballPos.x * 1000);
			out->set_y(ballPos.y * 1000);
			out->set_pixel_x(ballPos.x * 1000);
			out->set_pixel_y(ballPos.y * 1000);
		}
	}
	
	std::string buf;
	wrapper.SerializeToString(&buf);
	
	if (sendShared)
	{
		_visionSocket.writeDatagram(&buf[0], buf.size(), MulticastAddress, SharedVisionPort);
	} else {
		_visionSocket.writeDatagram(&buf[0], buf.size(), LocalAddress, SimVisionPort);
		_visionSocket.writeDatagram(&buf[0], buf.size(), LocalAddress, SimVisionPort + 1);
	}
}

void Env::convert_robot(const Robot *robot, SSL_DetectionRobot *out)
{
	Geometry2d::Point pos = robot->getPosition();
	out->set_confidence(1);
	out->set_robot_id(robot->shell);
	out->set_x(pos.x * 1000);
	out->set_y(pos.y * 1000);
	out->set_orientation(robot->getAngle() * DegreesToRadians);
	out->set_pixel_x(pos.x * 1000);
	out->set_pixel_y(pos.y * 1000);
}

void Env::addBall(Geometry2d::Point pos)
{
	Ball* b = new Ball(this);
	b->position(pos.x, pos.y);

	_balls.append(b);

	printf("New Ball: %f %f\n", pos.x, pos.y);
}

void Env::addRobot(bool blue, int id, Geometry2d::Point pos, Robot::Rev rev)
{
	Robot* r = new Robot(this, id, rev);
	r->position(pos.x, pos.y);

	if (blue)
	{
		_blue.insert(id, r);
	} else {
		_yellow.insert(id, r);
	}

	switch (rev) {
	case Robot::rev2008:
		printf("New 2008 Robot: %d : %f %f\n", id, pos.x, pos.y);
		break;
	case Robot::rev2010:
		printf("New 2010 Robot: %d : %f %f\n", id, pos.x, pos.y);
	}
}

void Env::removeRobot(bool blue, int id) {
	if (blue)
	{
		_blue.remove(id);
	} else {
		_yellow.remove(id);
	}
}

Geometry2d::Point gaussianPoint(int n, float scale)
{
	Geometry2d::Point pt;
	for (int i = 0; i < n; ++i)
	{
		pt.x += drand48() - 0.5;
		pt.y += drand48() - 0.5;
	}
	pt *= scale / n;

	return pt;
}

bool Env::occluded(Geometry2d::Point ball, Geometry2d::Point camera)
{
	float camZ = 4;
	float ballZ = Ball_Radius;
	float intZ = Robot_Height;

	// Find where the line from the camera to the ball intersects the
	// plane at the top of the robots.
	//
	// intZ = (ballZ - camZ) * t + camZ
	float t = (intZ - camZ) / (ballZ - camZ);
	Geometry2d::Point intersection;
	intersection.x = (ball.x - camera.x) * t + camera.x;
	intersection.y = (ball.y - camera.y) * t + camera.y;

	// Return true if the intersection point is inside any robot
	BOOST_FOREACH(const Robot* r, _blue)
	{
		if (intersection.nearPoint(r->getPosition(), Robot_Radius))
		{
			return true;
		}
	}
	BOOST_FOREACH(const Robot* r, _yellow)
	{
		if (intersection.nearPoint(r->getPosition(), Robot_Radius))
		{
			return true;
		}
	}
	return false;
}

Robot *Env::robot(bool blue, int board_id) const
{
	const QMap<unsigned int, Robot*> &robots = blue ? _blue : _yellow;
	
	if (robots.contains(board_id))
	{
		return robots[board_id];
	} else {
		return 0;
	}
}

void Env::handleRadioTx(int ch, const Packet::RadioTx& tx)
{
	// Channel 0 is yellow.
	// Channel 1 is blue.
	bool blue = ch;
	
	for (int i = 0; i < tx.robots_size(); ++i)
	{
		const Packet::RadioTx::Robot &cmd = tx.robots(i);
		
		if (cmd.motors_size() != 4)
		{
			printf("ch %d r %d: Wrong number of motors: %d != 4\n", ch, i, cmd.motors_size());
			continue;
		}
		
		Robot *r = robot(blue, cmd.robot_id());
		if (r)
		{
			r->radioTx(&cmd);
		} else {
			printf("Commanding nonexistant robot %s:%d\n",
				blue ? "Blue" : "Yellow",
				cmd.robot_id()); // FIXME: verify functionality
		}
	}
	
	// FIXME: radio protocol changed
//	Robot *rev = robot(blue, tx.reverse_board_id());
//	if (rev)
//	{
//		Packet::RadioRx rx = rev->radioRx();
//		rx.set_board_id(tx.reverse_board_id());
//
//		// Send the RX packet
//		std::string out;
//		rx.SerializeToString(&out);
//		_radioSocket[ch].writeDatagram(&out[0], out.size(), LocalAddress, RadioRxPort + ch);
//	}
}
