#include "Logger.hpp"

#include <stdio.h>
#include <boost/foreach.hpp>

using namespace Packet;

Logger::Logger()
{
	_history.resize(1000);
	_nextFrameNumber = 0;
}

void Logger::addFrame(const LogFrame& frame)
{
	QMutexLocker locker(&_mutex);
	
	_history[_nextFrameNumber % _history.size()] = frame;
	++_nextFrameNumber;
}

bool Logger::getFrame(int i, LogFrame& frame)
{
	QMutexLocker locker(&_mutex);
	
	if (_nextFrameNumber == 0 || i < (_nextFrameNumber - (int)_history.size()) || i >= _nextFrameNumber)
	{
		// Frame number is out of range
		return false;
	}
	
	frame.CopyFrom(_history[i % _history.size()]);
	
	return true;
}

int Logger::spaceUsed()
{
	QMutexLocker locker(&_mutex);
	
	int size = 0;
	BOOST_FOREACH(const LogFrame &frame, _history)
	{
		size += frame.SpaceUsed();
	}
	
	return size;
}
