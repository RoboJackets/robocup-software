#include "InputHandler.hpp"

#include <QMutex>
#include <QMutexLocker>

#include <Geometry/Point2d.hpp>

using namespace Geometry;

InputHandler::InputHandler() :
	QThread(), _running(true)
{
	_controller = new Gamepad("/dev/input/js0");
    _roller = 0;
    _rid = 0;

	_axels.push_back(Point2d(1,-1));
	_axels.push_back(Point2d(-1,-1));
	_axels.push_back(Point2d(-1,1));
	_axels.push_back(Point2d(1,1));
}

InputHandler::~InputHandler()
{
	delete _controller;

	_running = false;

	if (isRunning())
	{
		wait();
	}
}

/*
Packet::CommData::Robot InputHandler::genRobotData()
{
	Packet::CommData::Robot r;
	r.valid = true;

	float scale = 1.0f;
	int motors[4] = { 0, 0, 0, 0 };
        static uint8_t stored_roller = 0;

    //input is vx, vy in robot space
	Point2d input(_controller->rX(), _controller->rY());

	//if using DPad, this is the input value
	uint8_t mVal = 10 + (int8_t)(abs(_controller->rY()));

	if (_controller->dUp())
	{
		input.y = mVal;
		input.x = 0;
	}
	else if (_controller->dDown())
	{
		input.y = -mVal;
		input.x = 0;
	}
	else if (_controller->dRight())
	{
		input.y = 0;
		input.x = mVal;
	}
	else if (_controller->dLeft())
	{
		input.y = 0;
		input.x = -mVal;
	}

	int max = 0;

	for (unsigned int i=0; i<4; ++i)
	{
		motors[i] = (int)(_axels[i].perpCW().dot(input));
		motors[i] += _controller->lX();

		if (abs(motors[i]) > max)
		{
			max = abs(motors[i]);
		}
	}

	if (max > 127)
	{
		scale = 127.0f/max;
	}

	for (unsigned int i=0; i<4; ++i)
	{
		r.motor[i] = int8_t(scale * motors[i]);
	}

	if (_controller->b7())
	{
		_roller = _controller->lY();

        if (_controller->b5())
        {
            stored_roller = _controller->lY();
        }
    }

    if (_controller->b5())
    {
        _roller = stored_roller;
    }

    r.roller = _roller;

	if (_controller->b8())
	{
		r.kick = 255;
	}

	return r;
}
*/

void InputHandler::run()
{
	bool robotSelect = false;
	bool selectMode = false;

	while (_running)
	{
		if (_controller->waitForInput(100))
		{
			if (_controller->b9())
			{
				selectMode = true;

				unsigned int rid = 0;
				if (_controller->b1())
				{
					rid = 1;
				}
				else if (_controller->b2())
				{
					rid = 2;
				}
				else if (_controller->b3())
				{
					rid = 3;
				}
				else if (_controller->b4())
				{
					rid = 4;
				}

				if (rid > 0)
				{
                    _roller = 0;

                    if (rid != _rid)
                    {
                    	_rid = rid;
                    	printf("Robot Selected: %d\n", _rid);
                    	changeRobot(rid);
                    }
					robotSelect = true;
					continue;
				}
			}
			else if (selectMode)
			{
				//if no other robot selected default to 0
				if (!robotSelect && _rid)
				{
                    _roller = 0;
                    _rid = 0;
                    printf("Robot Selected: %d\n", _rid);
                    changeRobot(0);
				}

				robotSelect = false;
				selectMode = false;
				continue;
			}

			if (_controller->b4())
			{
                _roller = 0;
                playPauseButton();
			}
			else if (_controller->b3())
			{
                _roller = 0;
                manualAutoButton();
			}

			_continueProcessing.wakeAll();
		}
	}
}
