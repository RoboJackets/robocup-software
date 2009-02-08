#include "VisionGen.hpp"

#include <Network/Network.hpp>
#include "bin/Vision.hpp"

#include "Physics/Robot.hpp"

#include <QVector>
#include <Geometry/Point2d.hpp>
#include <Network/Sender.hpp>

#include <unistd.h>
#include <sys/time.h>

VisionGen::VisionGen(Env* env) :
	_env(env), _running(true)
{
    _id = 0;
    _fps = 30;
}

VisionGen::~VisionGen()
{
	_running = false;
	wait();
}

uint64_t VisionGen::timestamp()
{
	struct timeval time;
	gettimeofday(&time, 0);

	return time.tv_sec * 1000000 + time.tv_usec;
}

void VisionGen::run()
{
    Network::Sender sender(Network::Address, Network::Vision);
    
    //cycle time
    const int msecs = (int)(1000/_fps);
    
    while (_running)
    {
    	//send sync
    	Packet::Vision sync;
		sync.timestamp = timestamp();
		sync.sync = true;
		sender.send(sync);
	
		//get latest vision data from environment
		Packet::Vision genData = _env->vision();
		
		//fake vision processing time
		QThread::msleep(5);
		
		//TODO noise, multiple cameras
		genData.timestamp = timestamp();
		genData.camera = _id;
		sender.send(genData);
		
		//camera pause for cycle (minus fake processing time)
		QThread::msleep(msecs - 5);
    }
}


