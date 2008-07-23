#include "Vision.hpp"

#include <QVector>
#include <QUdpSocket>
#include <QTime>

#include <Packet/IO.hpp>
#include <Packet/VisionData.hpp>

using namespace Packet;

Vision::Vision(Physics::Env& env) :
	_env(env), _running(true), _ballStdDev(.002), _blueStdDev(.0015), _yellowStdDev(.0015)
{
	_error = false;
}

Vision::~Vision()
{
	_running = false;
	wait();
}

float Vision::err(const float stdDev)
{
	//check for degen case or error off
	if (stdDev == 0 || !_error)
	{
		return 0;
	}
	
	//TODO fix..oh jesus this sucks so bad
	while (true)
	{
		//between +-3*stdDev;
		float x = rand()/(float)RAND_MAX * 6 * stdDev - 3 * stdDev;
		
		float y = rand()/(float)RAND_MAX; //[0...1]
		
		const float expected_val = 0;
		float out = 1/(stdDev*sqrt(2*M_PI)) * exp(-(x-expected_val)*(x-expected_val)/(2*stdDev*stdDev));
		
		//continue until we get a y that works
		if (y <= out && x > -3*stdDev && x < 3*stdDev)
		{
			return x;
		}
	}
	
	//TODO maybe??
#if 0
	float a = rand()/(float)RAND_MAX;
	float b = rand()/(float)RAND_MAX;
	
	float c = sqrt(-2*log(a)) * cos(2*M_PI*b);
	float d = sqrt(-2*log(a)) * sin(2*M_PI*b);
#endif
}

float Vision::clip(float in, float min, float max)
{
	const float diff = max - min;
	if (in >= max)
	{
		return in - diff;
	}
	else if (in < min)
	{
		return in + diff;
	}
	
	return in;
}

void Vision::run()
{
	Sender<VisionData> yellowSendr(Yellow);
	Sender<VisionData> blueSendr(Blue);
	
	VisionData lastBlue;
	VisionData lastYellow;
	
	QTime time;
	time.start();
	
	while (_running)
	{
		int msec = time.restart();
		float sec = msec / 1000.0f;
		
		QVector<Physics::Robot*> yellow = _env.yellow();
		QVector<Physics::Robot*> blue = _env.blue();
		
		//populate vision data object
		VisionData blueData;
		VisionData yellowData;
		
		Physics::Ball* ball = _env.ball();
		if (ball)
		{
			const float bxerr = err(_ballStdDev);
			const float byerr = err(_ballStdDev);
			
			blueData.ball.valid = true;
			blueData.ball.pos.x = bxerr + ball->pos().y;
			blueData.ball.pos.y = FIELD_LENGTH/2.0f - ball->pos().x + byerr;
			
			if (lastBlue.ball.valid)
			{
				blueData.ball.vel = (lastBlue.ball.pos - blueData.ball.pos)/sec; 
			}
			
			yellowData.ball.valid = true;
			yellowData.ball.pos.x = -ball->pos().y + byerr;
			yellowData.ball.pos.y = ball->pos().x + bxerr + FIELD_LENGTH/2.0f;
			
			if (lastYellow.ball.valid)
			{
				yellowData.ball.vel = (yellowData.ball.pos - lastYellow.ball.pos)/sec; 
			}
		}
		
		//get blue team robot information
		Q_FOREACH(Physics::Robot* r, blue)
		{			
			const int id = r->id();
			
			const float xerr = err(_blueStdDev);
			const float yerr = err(_blueStdDev);
			const float terr = err(_blueStdDev);
			
			blueData.self[id].valid = true;
			blueData.self[id].pos.x = r->pos().y + xerr;
			blueData.self[id].pos.y = FIELD_LENGTH/2.0f - r->pos().x + yerr;
			blueData.self[id].theta = clip(r->theta() + terr - 90.0f, -180, 180);
			
			yellowData.opp[id].valid = true;
			yellowData.opp[id].pos.x = -r->pos().y + xerr;
			yellowData.opp[id].pos.y = r->pos().x + yerr + FIELD_LENGTH/2.0f;
			yellowData.opp[id].theta = clip(r->theta() + terr + 90.0f, -180, 180);
		}
		
		//get yellow team robot information
		Q_FOREACH(Physics::Robot* r, yellow)
		{
			const float xerr = err(_yellowStdDev);
			const float yerr = err(_yellowStdDev);
			const float terr = err(_yellowStdDev);
						
			const int id = r->id();
			
			blueData.opp[id].valid = true;
			blueData.opp[id].pos.x = r->pos().y + xerr;
			blueData.opp[id].pos.y = FIELD_LENGTH/2.0f - r->pos().x + yerr;
			blueData.opp[id].theta = clip(r->theta() - 90.0f + terr, -180, 180);
			
			yellowData.self[id].valid = true;
			yellowData.self[id].pos.x = -r->pos().y + xerr;
			yellowData.self[id].pos.y = r->pos().x + FIELD_LENGTH/2.0f + yerr;
			yellowData.self[id].theta = clip(r->theta() + 90.0f + terr, -180, 180);
		}
		
		//create velocities
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			if (lastBlue.self[i].valid && blueData.self[i].valid)
			{
				blueData.self[i].vel = (blueData.self[i].pos - lastBlue.self[i].pos)/sec;
			}
			
			if (lastBlue.opp[i].valid && blueData.opp[i].valid)
			{
				blueData.opp[i].vel = (blueData.opp[i].pos - lastBlue.opp[i].pos)/sec;
			}
			
			if (lastYellow.self[i].valid && yellowData.self[i].valid)
			{
				yellowData.self[i].vel = (yellowData.self[i].pos - lastYellow.self[i].pos)/sec;
			}
			
			if (lastYellow.opp[i].valid && yellowData.opp[i].valid)
			{
				yellowData.opp[i].vel = (yellowData.opp[i].pos - lastYellow.opp[i].pos)/sec;
			}
		}
		
		yellowData.timeNow();
		blueData.timeNow();
		
		lastBlue = blueData;
		lastYellow = yellowData;
		
		yellowSendr.send(yellowData);
		blueSendr.send(blueData);
		
		//QThread::msleep(33); //30 fps
		//QThread::msleep(16); //60 fps
		QThread::msleep(11); //90 fps
	}
}
