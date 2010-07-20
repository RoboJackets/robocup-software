#include "TestBallSpeed.hpp"
#include <boost/foreach.hpp>
#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestBallSpeed, "Tests")

Gameplay::Plays::TestBallSpeed::TestBallSpeed(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TestBallSpeed::applicable()
{
	return true;
}

bool Gameplay::Plays::TestBallSpeed::assign(set<Robot *> &available)
{
	for (int i = 0; i < Num_Speed_History; ++i)
	{
		_speed_history[i] = 0;
	}
	_last_pos = Geometry2d::Point();
	_first = true;
	_max_speed = 0;

	return true;
}

bool Gameplay::Plays::TestBallSpeed::run()
{
	if (!gameState().playing())
	{
		_max_speed = 0;
	}

	Geometry2d::Point ball;
	uint64_t time = 0;
	BOOST_FOREACH(Vision &vision, _gameplay->state()->rawVision)
	{
		if (!vision.balls.empty())
		{
			ball = vision.balls[0].pos;
			time = vision.timestamp;
			break;
		}
	}

	if (time)
	{
		_pos_history.push_back(ball);
		if (_pos_history.size() > 250)
		{
			_pos_history.pop_front();
		}
	}
	BOOST_FOREACH(Geometry2d::Point &pos, _pos_history)
	{
		state()->drawCircle(pos, Constants::Ball::Radius, Qt::red);
	}

	float speed = -1;
	float dtime = (time - _last_time) / 1000000.0;
	if (!_first && time && _last_time && dtime)
	{
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

	bool ok = true;
	float avg = 0;
	for (int i = 0; i < Num_Speed_History; ++i)
	{
		avg += _speed_history[i];
		ok &= _speed_history[i] >= 0;
	}
	avg /= Num_Speed_History;

	if (ok)
	{
		_max_speed = max(_max_speed, avg);
	}
	printf("%f %f\n", speed, _max_speed);

	return true;
}
