// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "InputHandler.hpp"
#include "InputHandler.moc"

#include <QMutex>
#include <QMutexLocker>

using namespace Geometry2d;

InputHandler::InputHandler(QObject* parent) :
	QThread(parent), _running(true)
{
	//try to open controller
	//if failed, prevents thread from running
	try
	{
		//TODO, need to check for vendor and device id
		//_controller = new Gamepad("/dev/input/js0");
		_controller = new Gamepad("/dev/input/robocupPad");
	}
	catch (std::runtime_error& re)
	{
		printf("InputHandler Error: %s\n", re.what());
		printf("Not using joystick controller\n");
		_controller = 0;
		_running = false;
	}

	_roller = 0;

	_axles.push_back(Point(1, -1));
	_axles.push_back(Point(-1, -1));
	_axles.push_back(Point(-1, 1));
	_axles.push_back(Point(1, 1));
}

InputHandler::~InputHandler()
{
	delete _controller;

	_running = false;
	wait();
}

void InputHandler::genRobotData(Packet::RadioTx::Robot &tx)
{
	tx.valid = true;

	float scale = 1.0f;
	int motors[4] =	{ 0, 0, 0, 0 };
	static uint8_t stored_roller = 0;
	
	//input is vx, vy in robot space
	Point input(_controller->rX(), _controller->rY());

	//if using DPad, this is the input value
	uint8_t mVal = 10 + (int8_t) (abs(_controller->rY()));

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

	for (unsigned int i = 0; i < 4; ++i)
	{
		motors[i] = (int) (_axles[i].perpCW().dot(input));
		motors[i] -= _controller->lX();

		if (abs(motors[i]) > max)
		{
			max = abs(motors[i]);
		}
	}

	if (max > 127)
	{
		scale = 127.0f / max;
	}

	for (unsigned int i = 0; i < 4; ++i)
	{
		tx.motors[i] = int8_t(scale * motors[i]);
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

	tx.roller = _roller;

	if (_controller->b8())
	{
		tx.kick = 255;
	}
}

void InputHandler::run()
{
	enum
	{
		B9_Released,
		B9_Pressed,
		Selected
	} selectState = B9_Released;
	
	while (_running)
	{
		if (_controller->waitForInput(100))
		{
			_roller = 0;
			
			if (_controller->b9())
			{
				if (selectState == B9_Released)
				{
					selectState = B9_Pressed;
				}
				
				if (_controller->b1())
				{
					selectState = Selected;
					selectRobot(1);
				}
				else if (_controller->b2())
				{
					selectState = Selected;
					selectRobot(2);
				}
				else if (_controller->b3())
				{
					selectState = Selected;
					selectRobot(3);
				}
				else if (_controller->b4())
				{
					selectState = Selected;
					selectRobot(4);
				}
			} else {
				if (selectState == B9_Pressed)
				{
					// B9 was pressed and released without pressing 1-4.
					selectRobot(0);
				}
				selectState = B9_Released;
				
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
			}
			//_continueProcessing.wakeAll();
		}
	}
}
