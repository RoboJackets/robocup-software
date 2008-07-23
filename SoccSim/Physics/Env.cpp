#include "Env.hpp"

#include <QTime>

#include <Geometry/Point2d.hpp>

using namespace Physics;
using namespace Geometry;

Packet::CommData Env::_blueData;
Packet::CommData Env::_yellowData;

Env::Env() :
	_receiver(this)
{
	//clear the teams and ball
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_yellow[i] = _blue[i] = 0;
	}
	_ball = 0;

	dInitODE();

	_world = dWorldCreate();
	//dWorldSetCFM (_world, 10e-7);

	//earth's gravity
	dWorldSetGravity(_world, 0, 0, -9.81);

	//setup ode space and contact group
	_space = dSimpleSpaceCreate(0);
	_contactGroup = dJointGroupCreate(0);

	_running = true;
	_paused = false;

	//add a field
	_entities.append(new Field(_world, _space));
}

Env::~Env()
{
	_receiver.terminate();
	_receiver.wait();

	_running = false;
	wait();

	Q_FOREACH(Entity* e, _entities)
	{
		delete e;
	}

	dWorldDestroy(_world);
	dSpaceDestroy(_space);
	dCloseODE();
}

void Env::EnvReceiver::run()
{
	
	Packet::PacketReceiver _receiver;

	//add comm receiver handlers
	_receiver.addType(Blue, Env::commBlue, 100);
	_receiver.addType(Yellow, Env::commYellow, 100);
	
	_receiver.addType(Blue, this, &EnvReceiver::simCmdHandler);
	_receiver.addType(Yellow, this, &EnvReceiver::simCmdHandler);
	
	while (true)
	{
		_receiver.receive();
	}
}

void Env::commYellow(const Packet::CommData* data)
{
	if (data)
	{
		Env::_yellowData = *data;
	}
	else
	{
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			Env::_yellowData.robots[i].valid = false;
		}
	}
}

void Env::commBlue(const Packet::CommData* data)
{
	if (data)
	{
		Env::_blueData = *data;
	}
	else
	{
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			Env::_blueData.robots[i].valid = false;
		}
	}
}

void Env::EnvReceiver::simCmdHandler(const Packet::SimCmd* data)
{
	if (_env->_ball)
	{
		if (data->setBallPos)
		{
			_env->_ball->setPosition(data->ballPos.x, data->ballPos.y, 0);
		}
		_env->_ball->setVelocity(data->ballVel.x, data->ballVel.y, 0);
	}
}

QVector<Robot*> Env::blue()
{
	QMutexLocker ml(&_entitiesMutex);
	QVector<Robot*> robots;

	for (unsigned int i=0; i<5; ++i)
	{
		if (_blue[i])
		{
			robots.append(_blue[i]);
		}
	}

	return robots;
}

QVector<Robot*> Env::yellow()
{
	QMutexLocker ml(&_entitiesMutex);
	QVector<Robot*> robots;

	for (unsigned int i=0; i<5; ++i)
	{
		if (_yellow[i])
		{
			robots.append(_yellow[i]);
		}
	}

	return robots;
}

void Env::controlRobot(Physics::Robot* envR, const Packet::CommData::Robot& r)
{
	if (r.valid && envR)
	{
		//use packet information
		envR->motors(r.motor[0], r.motor[1], r.motor[2], r.motor[3]);
		envR->kick(r.kick);
		envR->roller(r.roller);
	}
	else if (envR)
	{
		//turn everything off on the robot
		envR->motors(0, 0, 0, 0);
		envR->kick(0);
		envR->roller(0);
	}
}

void Env::addRobot(Team t, unsigned int id)
{
	Point2d pos(1, FLOOR_WIDTH/2.0-.12);
	int sign = 1;
	
	QMutexLocker ml(&_entitiesMutex);

	id = id%5;
	Robot* r = new Robot(_world, _space, id);

	Robot** teamArr;
	if (t == Blue)
	{
		teamArr = _blue;
	}
	else
	{
		teamArr = _yellow;
		pos.x = -1;
		sign = -1;
	}

	if (!teamArr[id])
	{
		teamArr[id] = r;
		_entities.append(r);
	}
	else
	{
		delete r;
		return;
	}

	bool clear = false;
	while (!clear)
	{
		clear = true;
		for (int i=0; i<_entities.size() ; ++i)
		{
			if (_entities[i]->pos().distTo(pos) < ROBOT_DIAM)
			{
				clear = false;
				pos.x += sign * (ROBOT_DIAM + .05);
			}
		}
	}

	r->setPosition(pos.x, pos.y);
}

void Env::removeRobot(Team t, unsigned int id)
{
	QMutexLocker ml(&_entitiesMutex);

	Robot* r = 0;

	if (t == Blue && _blue[id])
	{
		r = _blue[id];
		_blue[id] = 0;
	}
	else if (_yellow[id])
	{
		r = _yellow[id];
		_yellow[id] = 0;
	}

	int index = _entities.indexOf(r);
	if (index >= 0)
	{
		_entities.remove(index);
	}

	if (r)
	{
		delete r;
	}
}

void Env::run()
{
	//start the receiver
	_receiver.start();

	//ode step timer
	QTime time;
	time.start();
	
	Packet::PacketSender yellowSender(Yellow);
	Packet::PacketSender blueSender(Blue);
	
	Packet::RobotStatus yellowRobotStatus, blueRobotStatus;

	while (_running)
	{
		_entitiesMutex.lock();

		for (unsigned int i=0; i<5; ++i)
		{
			controlRobot(_blue[i], Env::_blueData.robots[i]);
			controlRobot(_yellow[i], Env::_yellowData.robots[i]);
		}

		Q_FOREACH(Physics::Entity* e, _entities)
		{
			e->internal();
		}

		const float step = time.restart()/1000.0f;

		dSpaceCollide(_space, this, Physics::Env::nearCollision);
		dWorldStep(_world, step);
		dJointGroupEmpty(_contactGroup);
		
		//populate ball presense
		for (unsigned int i=0; i<5; ++i)
		{
			if (_blue[i])
			{
				blueRobotStatus.robots[i].ballPresent = _blue[i]->ballPresent();
				blueRobotStatus.robots[i].charged = _blue[i]->charged();
			}
			
			if (_yellow[i])
			{
				yellowRobotStatus.robots[i].ballPresent = _yellow[i]->ballPresent();
				yellowRobotStatus.robots[i].charged = _yellow[i]->charged();
			}
		}
		
		_entitiesMutex.unlock();
		
		yellowSender.send(yellowRobotStatus);
		blueSender.send(blueRobotStatus);

		QThread::msleep(5);
	}
}

void Env::enableBall()
{
	QMutexLocker ml(&_entitiesMutex);

	_ball = new Ball(_world, _space);
	_entities.append(_ball);
}

void Env::disableBall()
{
	QMutexLocker ml(&_entitiesMutex);

	if (_ball)
	{
		//remove from entities
		int index = _entities.indexOf(_ball);
		if (index >= 0)
		{
			_entities.remove(index);
		}

		delete _ball;
		_ball = 0;
	}
}

void Env::resetRobot(Team t, unsigned int id)
{
	//TODO
}

void Env::resetBall()
{
	//TODO
}

void Env::pause(bool p)
{
	//TODO
}

/// ODE stuff ///

void Env::nearCollision(void* _this, dGeomID o1, dGeomID o2)
{
	Env* env = (Env*)_this;

	dBodyID body1 = dGeomGetBody(o1);
	dBodyID body2 = dGeomGetBody(o2);

	Entity::Geom g1 = (Entity::Geom)(long)dGeomGetData(o1);
	Entity::Geom g2 = (Entity::Geom)(long)dGeomGetData(o2);

	unsigned int collisions = 0;
	dContact* contact = 0;

	if (g1 == Entity::RobotShell && g2 == Entity::Floor)
	{
		collisions = 10;
		contact = new dContact[collisions];

		collisions = dCollide(o1, o2, collisions, &contact[0].geom, sizeof(dContact));

		for (unsigned int i=0 ; i<collisions ; ++i)
		{
			contact[i].surface.slip1 = .1;
			contact[i].surface.slip2 = .1;
			contact[i].surface.mode = dContactSlip1 | dContactSlip2;
			contact[i].surface.mu = 1000.0;
		}
	}
	else if (g1 == Entity::RobotShell && g2 == Entity::RobotShell)
	{
		const dReal* p1 = dBodyGetPosition(body1);
		const dReal* p2 = dBodyGetPosition(body2);

		Point2d pt1(p1[0], p1[1]), pt2(p2[0], p2[1]);
		Point2d norm = pt1 - pt2;

		if (norm.mag() <= 2*ROBOT_RADIUS)
		{
			collisions = 1;
			contact = new dContact[collisions];

			contact[0].surface.mode = dContactBounce | dContactFDir1;
			contact[0].surface.bounce = .3f;
			contact[0].surface.mu = 1.0f;

			contact[0].geom.normal[0] = norm.x;
			contact[0].geom.normal[1] = norm.y;
			contact[0].geom.normal[2] = 0;

			contact[0].geom.depth = 2*ROBOT_RADIUS - norm.mag();

			norm = norm.norm();
			norm *= ROBOT_RADIUS;

			contact[0].geom.pos[0] = pt1.x + norm.x;
			contact[0].geom.pos[1] = pt1.y + norm.y;
			contact[0].geom.pos[2] = 0;

			contact[0].fdir1[0] = norm.y;
			contact[0].fdir1[1] = -norm.x;
			contact[0].fdir1[2] = 0;
		}
	}
	else
	{
		collisions = 10;
		contact = new dContact[collisions];

		collisions = dCollide(o1, o2, collisions, &contact[0].geom, sizeof(dContact));

		float bounce = 0;
		float mu = 0;

		if (g1 == Entity::Ball && g2 == Entity::Floor)
		{
			bounce = .1;
			mu = 0.05;
		}
		else if (g1 == Entity::Roller && g2 == Entity::Ball)
		{
			bounce = .1;
			mu = 20.0f;
			
			Robot* r = (Robot*)dBodyGetData(body1);
			r->ballPresent(true);
		}
		else if (g1 == Entity::Ball && g2 == Entity::Roller)
		{
			bounce = .1;
			mu = 20.0f;
			
			Robot* r = (Robot*)dBodyGetData(body2);
			r->ballPresent(true);
		}
		else if ((g1 == Entity::Kicker && g2 == Entity::Ball) ||
				(g1 == Entity::Ball && g2 == Entity::Kicker))
		{
			bounce = .9;
			mu = 0.0f;
		}
		else if (g1 == Entity::Ball && g2 == Entity::Wall)
		{
			bounce = .4;
			mu = 0.0f;
		}
		else if (g1 == Entity::RobotShell && g2 == Entity::Wall)
		{
			bounce = .05;
			mu = 1.0f;
		}
		else
		{
			//not recognized...no contact
			collisions = 0;
		}

		for (unsigned int i=0 ; i<collisions ; ++i)
		{
			contact[i].surface.mode = dContactBounce;
			contact[i].surface.bounce = bounce;
			contact[i].surface.mu = mu;
		}
	}

	//attach the contact joints
	for (unsigned int i = 0; i < collisions; ++i)
	{
		dJointID c = dJointCreateContact(env->_world, env->_contactGroup, &contact[i]);
		dJointAttach(c, body1, body2);
	}

	if (contact)
	{
		delete[] contact;
	}
}
