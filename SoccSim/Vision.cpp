#include "Vision.hpp"

#include <Network/Network.hpp>
#include "bin/Vision.hpp"

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

    QVector<Geometry::Point2d*> robotsPositions;

    while (true)
    {
        //clear all previous
	packet.blue.clear();
	packet.yellow.clear();
	packet.balls.clear();
        robotsPositions.clear();

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

        robotsPositions = _env->getRobotsPositions();
        int i=0;
        Q_FOREACH(Geometry::Point2d* pos, robotsPositions)
        {
            r.shell = i++;
	    r.pos = *pos;
            packet.blue.push_back(r);
        }

	sender.send(packet);
	//camera pause
	QThread::msleep(msecs - 5);
    }
}


