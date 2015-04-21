#include "PreventDoubleTouch.hpp"

#include <stdio.h>


using namespace std;

Gameplay::PreventDoubleTouch::PreventDoubleTouch(GameplayModule *gameplay, SingleRobotBehavior *kicker):
	backoff(gameplay)
{
	_gameplay = gameplay;
	_kicker = kicker;
	_keepRunning = false;
	_kicked = false;
	_wasReady = false;
}

void Gameplay::PreventDoubleTouch::run()
{
	const GameState &gameState = state()->gameState;
	const Ball &ball = state()->ball;
	
	if (!_kicker || !_kicker->robot)
	{
		// Don't get stuck in a play if we don't have a kicking robot
		_keepRunning = false;
		return;
	}
	
	if (_keepRunning && (gameState.state != GameState::Ready && gameState.state != GameState::Playing))
	{
		// We were waiting for another robot to touch the ball but the referee has cancelled the kickoff.
		// Allow applicable() to return false.
		_keepRunning = false;
	}
	
	if (gameState.state == GameState::Ready)
	{
		// At this point the _kicker should either kick or collide with the ball.
		// After that one other robot must touch it.
		_keepRunning = true;
		_wasReady = true;
	}
	
	if (_kicked)
	{
		backoff.run();
		if (_ruleTime.msecsTo(QTime::currentTime()) > 5000)
		{
			_keepRunning = false;
		}
	} else {
		if ((_kicker->robot->visible && !_kicker->run()) || (gameState.state == GameState::Playing && _wasReady))
		{
			_kicked = true;
			_ruleTime = QTime::currentTime();
		}
	}
	
	Robot *best = 0;
	float bestDist = 0;
	for (Robot *r :  state()->self)
	{
		if (r->visible)
		{
			float dist = r->pos.distTo(ball.pos);
			if (bestDist == 0 || bestDist > dist)
			{
				bestDist = dist;
				best = r;
			}
		}
	}
	if (best != _kicker->robot)
	{
		// Another robot is closer to the ball, so we're done with this play.
		// Allow applicable to return false on the next frame.
		_keepRunning = false;
	}
}
