#include "SimVision.hpp"

#include <unistd.h>
#include <sys/time.h>
#include <Network/Sender.hpp>
#include <Vision.hpp>

SimVision::SimVision(Env* env) :
	_env(env)
{
    //TODO make it so that these can be derived from config file
    _id = 0;
    _fps = 30;
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
    //Network::Sender sender(Network::Address, Network::Vision);

    //Packet::Vision packet;
    //packet.camera = _id;

    //Packet::Vision::Robot r = Packet::Vision::Robot();
    /*
    r.pos.x = -2;
    r.pos.y = 0;
    */
    const int msecs = (int)(1000/_fps);

    //float rad = 0;

    //loop and send out robot positions
    while (true)
    {
	//clear all previous
// 	packet.blue.clear();
// 	packet.yellow.clear();
// 	packet.balls.clear();

	//send sync
// 	packet.timestamp = timestamp();
// 	packet.sync = true;
// 	sender.send(packet);

	//fake vision processing
	QThread::msleep(5);

	//send real data
// 	packet.sync = false;
// 	packet.timestamp = timestamp();

        //get the current positions of the robots
        /*
	rad += .025;

	if (rad > 2*M_PI)
	{
		rad -= 2*M_PI;
	}

	r.shell = _id;

	#if 1
	r.pos.x = _sx + cos(rad);
	r.pos.y = _sy + sin(rad);
	#else
	r.pos.x += .1;
	r.pos.y = 0;

	if (r.pos.x >= 2)
	{
		r.pos.x = -2;
	}
	#endif
	r.angle = 45.0f * _id;

	packet.blue.push_back(r);
        */
	//send vision data
	//sender.send(packet);

// 	printf("[%d] %lld :: Pos: %.3f %.3f\n", _id, packet.timestamp, r.pos.x, r.pos.y);

	//camera pause
	QThread::msleep(msecs - 5);
    }
}
