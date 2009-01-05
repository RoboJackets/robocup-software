#include "SimVision.hpp"

#include <unistd.h>
#include <sys/time.h>
#include <QVector>
#include <Geometry/Point2d.hpp>
#include <Network/Sender.hpp>
#include <Network/Network.hpp>
#include <Vision.hpp>

SimVision::SimVision(Env* env) :
	_env(env)
{
    //TODO make it so that these can be derived from config file
    _id = 0;
    _fps = 30;
    _receiver = new Network::PacketReceiver();
    _receiver->addType(Network::Address, Network::addTeamOffset(Blue,Network::RadioTx), this, &SimVision::radioHandler);
}

SimVision::~SimVision()
{

}

uint64_t SimVision::timestamp()
{
	struct timeval time;
	gettimeofday(&time, 0);

	return time.tv_sec * 1000000 + time.tv_usec;
}

void SimVision::run()
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

        _simVisionMutex.lock();
        robotsPositions = _env->getRobotsPositions();

        Q_FOREACH(Geometry::Point2d* pos, robotsPositions)
        {
	    r.pos = *pos;
            packet.blue.push_back(r);
        }

	sender.send(packet);

        _simVisionMutex.unlock();

        _receiver->receive();
	//camera pause
	QThread::msleep(msecs - 5);
    }
}

void SimVision::radioHandler(const Packet::RadioTx* packet)
{
    _env->txPacket = packet;
    //printf("Packet Rxd\n");
}

