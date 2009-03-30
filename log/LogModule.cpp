#include "LogModule.hpp"

#include <boost/foreach.hpp>

#include "drawing/Elements.hpp"

using namespace Log;

LogModule::LogModule() :
	Module("Log Module")
{
	_logFile = 0;
}

void LogModule::setLogFile(LogFile* file)
{
    //TODO this mutex should not be here...instread in code
    //that actually uses this object?? - Roman
	_logFileMutex.lock();
	_logFile = file;
	_logFileMutex.unlock();
}

void LogModule::fieldOverlay(QPainter& p, Packet::LogFrame& f) const
{
	BOOST_FOREACH(const Packet::Vision& vision, f.rawVision)
	{
		if (vision.sync)
		{
			return;
		}
		
		BOOST_FOREACH(const Packet::Vision::Robot& r, vision.blue)
		{
			drawRobot(p, Blue, r.shell, r.pos, r.angle);
		}
		
		BOOST_FOREACH(const Packet::Vision::Robot& r, vision.yellow)
		{
			drawRobot(p, Yellow, r.shell, r.pos, r.angle);
		}
		
		BOOST_FOREACH(const Packet::Vision::Ball& b, vision.balls)
		{
			drawBall(p, b.pos);
		}
	}
}

void LogModule::run()
{
	_logFileMutex.lock();
	if (_logFile && _state)
	{
		//write out log frame part of the state
		_logFile->write(*_state);
	}
	_logFileMutex.unlock();
	
}
