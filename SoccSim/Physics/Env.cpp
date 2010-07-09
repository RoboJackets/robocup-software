#include "Env.hpp"
#include "Env.moc"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>

#include <sys/time.h>
#include <Constants.hpp>
// #include <QMutexLocker>
#include <boost/foreach.hpp>
#include <Geometry2d/util.h>

using namespace Geometry2d;

const QHostAddress VisionAddress("224.5.23.2");
const int VisionPort = 10002;

Env::Env()
{
	_frameNumber = 0;
	
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

	//always add the field
	_field = new Field(this);

	//NxActor** actors = _scene->getActors();
	//NxU32 actorCount = _scene->getNbActors();

	//packetRxd = 0;
	//_receiver = new Network::PacketReceiver();
	//_receiver->addType(Network::Address, Network::addTeamOffset(Blue,Network::RadioTx), this, &Env::radioHandler);
	//txPacket = new Packet::RadioTx();

	gettimeofday(&_lastStepTime, 0);
	
	connect(&_timer, SIGNAL(timeout()), SLOT(step()));
	_timer.start(5);
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
// 	QMutexLocker ml(&_sceneMutex);
	return *_scene->getDebugRenderable();
}

void Env::step()
{
	//FIXME - Check sockets
	
	// Run physics
	struct timeval tv;
	gettimeofday(&tv, 0);
// 	_sceneMutex.lock();
	_scene->simulate(tv.tv_sec - _lastStepTime.tv_sec + (tv.tv_usec - _lastStepTime.tv_usec) * 1.0e-6);
	_lastStepTime = tv;
	_scene->flushStream();
// 	_sceneMutex.unlock();
	_scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

	++_stepCount;
	if (_stepCount == 4)
	{
		_stepCount = 0;
		
		// Send vision data
		SSL_WrapperPacket packet;
		SSL_DetectionFrame *det = packet.mutable_detection();
		det->set_frame_number(_frameNumber++);
		det->set_camera_id(0);
		
		det->set_t_capture(tv.tv_sec + (double)tv.tv_usec * 1.0e-6);
		det->set_t_sent(det->t_capture());
		
		BOOST_FOREACH(Robot *robot, _yellow)
		{
			SSL_DetectionRobot *out = det->add_robots_yellow();
			convert_robot(robot, out);
		}
		
		BOOST_FOREACH(Robot *robot, _blue)
		{
			SSL_DetectionRobot *out = det->add_robots_blue();
			convert_robot(robot, out);
		}
		
		Geometry2d::Point cam0(-Constants::Field::Length / 4, 0);
		Geometry2d::Point cam1(Constants::Field::Length / 4, 0);

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

			if (!occ)
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
		packet.SerializeToString(&buf);
		_visionSocket.writeDatagram(&buf[0], buf.size(), VisionAddress, VisionPort);
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
// 	QMutexLocker ml1(&_sceneMutex);
// 	QMutexLocker ml2(&_entitiesMutex);

	Ball* b = new Ball(this);
	b->position(pos.x, pos.y);

	_balls.append(b);

	printf("New Ball: %f %f\n", pos.x, pos.y);
}

void Env::addRobot(Team t, int id, Geometry2d::Point pos, Robot::Rev rev)
{
	if (t == UnknownTeam)
	{
		printf("Cannot add robots to unknown team.\n");
		return;
	}

// 	QMutexLocker ml1(&_sceneMutex);
// 	QMutexLocker ml2(&_entitiesMutex);

	Robot* r = new Robot(this, id, rev);
	r->position(pos.x, pos.y);

	if (t == Blue)
	{
		_blue.insert(id, r);
	}
	else
	{
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
	float ballZ = Constants::Ball::Radius;
	float intZ = Constants::Robot::Height;

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
		if (intersection.nearPoint(r->getPosition(), Constants::Robot::Radius))
		{
			return true;
		}
	}
	BOOST_FOREACH(const Robot* r, _yellow)
	{
		if (intersection.nearPoint(r->getPosition(), Constants::Robot::Radius))
		{
			return true;
		}
	}
	return false;
}

// Packet::RadioRx Env::radioRx(Team t)
// {
// 	QMutexLocker ml(&_radioRxMutex);
// 
// 	if (t == Blue)
// 	{
// 		return _radioRxBlue;
// 	}
// 	else if (t == Yellow)
// 	{
// 		return _radioRxYellow;
// 	}
// 
// 	return Packet::RadioRx();
// }

Robot *Env::robot(Team t, int board_id) const
{
	const QMap<unsigned int, Robot*> &robots = (t == Blue) ? _blue : _yellow;
	
	if (robots.contains(board_id))
	{
		return robots[board_id];
	} else {
		return 0;
	}
}

// void Env::radioTx(Team t, const Packet::RadioTx& tx)
// {
// 	//control robots when not working with the scene
// 	//this will prep new data for the next scene
// 	QMutexLocker ml(&_sceneMutex);
// 
// 	for (int i = 0; i < 5; ++i)
// 	{
// 		const Packet::RadioTx::Robot& cmd = tx.robots[i];
// 		if (cmd.valid)
// 		{
// 			Robot *r = robot(t, cmd.board_id);
// 			if (r)
// 			{
// 				r->radioTx(cmd);
// 			} else {
// 				printf("Commanding nonexistant robot %s:%d\n",
// 					(t == Blue) ? "Blue" : "Yellow",
// 					cmd.board_id);
// 			}
// 		}
// 	}
// 	
// 	Robot *rev = robot(t, tx.reverse_board_id);
// 	if (rev)
// 	{
// 		Packet::RadioRx rx = rev->radioRx();
// 		rx.board_id = tx.reverse_board_id;
// 		
// 		if (t == Blue)
// 		{
// 			_radioRxBlue = rx;
// 		} else {
// 			_radioRxYellow = rx;
// 		}
// 	}
// }

// void Env::command(const Packet::SimCommand &cmd)
// {
//     QMutexLocker ml(&_sceneMutex);
// 
//     if (cmd.ball.valid && !_balls.empty())
//     {
//         _balls[0]->velocity(cmd.ball.vel.x, cmd.ball.vel.y);
//         _balls[0]->position(cmd.ball.pos.x, cmd.ball.pos.y);
//     }
// }
