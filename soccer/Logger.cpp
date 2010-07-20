#include "Logger.hpp"

Logger::Logger()
{
	_history.resize(100);
	_nextPos = 0;
}

void Logger::addFrame(const Packet::LogFrame& frame)
{
	QMutexLocker locker(&_mutex);
	
	_history[_nextPos] = frame;
	_nextPos = (_nextPos + 1) % _history.size();
}
