#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdlib.h>
#include <vector>

#include <QThread>

#include <Vision.hpp>
#include <Network/Sender.hpp>
#include <Network/Network.hpp>

uint64_t timestamp()
{
	struct timeval time;
	gettimeofday(&time, 0);

	return time.tv_sec * 1000000 + time.tv_usec;
}

class VisionSender : public QThread
{
	public:
		VisionSender(float sx, float sy, unsigned int fps)
		{
			_id = _nextId++;
			_sx = sx;
			_sy = sy;

			_fps = fps;

			printf("Creating fake camera with id: %d\n", _id);
			printf("Robot start @ (%f , %f) w/ fps: %d\n", _sx, _sy, _fps);
		}

	protected:
		void run()
		{
			Network::Sender sender(Network::Address, Network::Vision);

			Packet::Vision packet;
			packet.camera = _id;

			Packet::Vision::Robot r = Packet::Vision::Robot();
			r.pos.x = -2;
			r.pos.y = 0;

			const int msecs = (int)(1000/_fps);

			float rad = 0;

			//loop and send out robot positions
			while (true)
			{
				//clear all previous
				packet.blue.clear();
				packet.yellow.clear();
				packet.balls.clear();

				//send sync
				packet.timestamp = timestamp();
				packet.sync = true;
				sender.send(packet);

				//fake vision processing
				QThread::msleep(5);

				//send real data
				packet.sync = false;
				packet.timestamp = timestamp();

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

				//send vision data
				sender.send(packet);

				printf("[%d] %lld :: Pos: %.3f %.3f\n", _id, packet.timestamp, r.pos.x, r.pos.y);

				//camera pause
				QThread::msleep(msecs - 5);
			}
		}

	private:
		unsigned int _id;

		float _sx, _sy;

		unsigned int _fps;

		static unsigned int _nextId;
};

unsigned int VisionSender::_nextId = 0;

int main(int argc, const char* argv[])
{
	if (argc != 3)
	{
		printf("Usage: %s <cam count> <fps>\n", argv[0]);
		return 0;
	}

	int numCameras = atoi(argv[1]);
	unsigned int fps = atoi(argv[2]);

	if (numCameras < 1)
	{
		numCameras = 1;
	}

	float x = 2.0f;
	float y = 0;

	float inc = 4.0f;

	if (numCameras > 1)
	{
		inc /= (numCameras - 1);
	}

	std::vector<VisionSender*> cameras;

	for (int i=0 ; i<numCameras ; ++i)
	{
		cameras.push_back(new VisionSender(x, y, fps));
		x -= inc;
	}

	printf("---------------------------------------------------------\n");
	printf("Sending data...\n");

	int msecs = (int)(1000/fps);

	for (unsigned int i=0 ; i<cameras.size() ; ++i)
	{
		cameras[i]->start();

		//delay until start of next camera
		msecs *= .45;
		usleep(1000 * msecs);
	}

	cameras[0]->wait();

	return 0;
}
