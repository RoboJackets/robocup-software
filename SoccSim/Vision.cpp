#include "Vision.hpp"

#include <Network/Network.hpp>
#include "bin/Vision.hpp"

#include "Physics/Robot.hpp"

#include <QVector>
#include <Geometry/Point2d.hpp>
#include <Network/Sender.hpp>

#include <unistd.h>
#include <sys/time.h>

Vision::Vision(Env* env) :
	_env(env)
{
    _id = 0;
    _fps = 30;
}

Vision::~Vision()
{

}

uint64_t Vision::timestamp()
{
	struct timeval time;
	gettimeofday(&time, 0);

	return time.tv_sec * 1000000 + time.tv_usec;
}

void Vision::run()
{
    Network::Sender sender(Network::Address, Network::Vision);

    Packet::Vision packet;
    packet.camera = _id;

    Packet::Vision::Robot r = Packet::Vision::Robot();
    const int msecs = (int)(1000/_fps);

    QVector<Robot*> robots;

    while (true)
    {
    	//clear all previous
		packet.blue.clear();
		packet.yellow.clear();
		packet.balls.clear();
		robots.clear();
	
		//send sync
		packet.timestamp = timestamp();
		packet.sync = true;
		sender.send(packet);
	
		//fake vision processing
		QThread::msleep(5);
	
		//send real data
		packet.sync = false;
		packet.timestamp = timestamp();
	
		//get the current positions of the robots
	
		robots = _env->getRobots();
		int i=0;
		
		
		//TODO blue and yellow separate...
		
		Q_FOREACH(Robot* robot, robots)
		{
			r.shell = i++;
			r.pos =robot->getPosition(); 
			r.angle = robot->getAngle(); 
			packet.blue.push_back(r);
		}
	
		sender.send(packet);
		//camera pause
		QThread::msleep(msecs - 5);
    }
}


