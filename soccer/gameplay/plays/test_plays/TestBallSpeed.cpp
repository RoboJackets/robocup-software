#include "TestBallSpeed.hpp"
#include <boost/foreach.hpp>

using namespace std;

Gameplay::Plays::TestBallSpeed::TestBallSpeed(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TestBallSpeed::applicable()
{
	return true;
}

void Gameplay::Plays::TestBallSpeed::assign(set<Robot *> &available)
{
	for (int i = 0; i < Num_Speed_History; ++i)
	{
		_speed_history[i] = 0;
	}
	_last_pos = Geometry2d::Point();
	_first = true;
	_max_speed = 0;
}

bool Gameplay::Plays::TestBallSpeed::run()
{
	Geometry2d::Point ball;
	uint64_t time = 0;
	BOOST_FOREACH(Packet::Vision &vision, _gameplay->state()->rawVision)
	{
		if (!vision.balls.empty())
		{
			ball = vision.balls[0].pos;
			time = vision.timestamp;
			break;
		}
	}
	if (!time)
	{
		return true;
	}

	float speed = 0;
	if (!_first)
	{
		float dtime = (time - _last_time) / 1000000.0;
		speed = (ball - _last_pos).mag() / dtime;
	}
	_first = false;
	_last_pos = ball;
	_last_time = time;

	for (int i = 1; i < Num_Speed_History; ++i)
	{
		_speed_history[i - 1] = _speed_history[i];
	}
	_speed_history[Num_Speed_History - 1] = speed;

	float avg = 0;
	for (int i = 0; i < Num_Speed_History; ++i)
	{
		avg += _speed_history[i];
	}
	avg /= Num_Speed_History;

	if (!gameState().playing())
	{
		_max_speed = 0;
	}
	_max_speed = max(_max_speed, avg);
	printf("%f\n", _max_speed);

	return true;
}
