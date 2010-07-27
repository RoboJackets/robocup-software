#include "Logger.hpp"

#include <QString>
#include <boost/foreach.hpp>

using namespace std;
using namespace Packet;

Logger::Logger()
{
	_history.resize(100000, 0);
	_nextFrameNumber = 0;
	_spaceUsed = sizeof(_history[0]) * _history.size();
}

Logger::~Logger()
{
	// Delete frames in history
	BOOST_FOREACH(Packet::LogFrame *frame, _history)
	{
		delete frame;
	}
}

bool Logger::open(QString filename)
{
	QMutexLocker locker(&_mutex);
	_file.open((const char *)filename.toAscii(), ofstream::out);
	
	return _file.is_open();
}

void Logger::close()
{
	QMutexLocker locker(&_mutex);
	_file.close();
}

void Logger::addFrame(const LogFrame& frame)
{
	QMutexLocker locker(&_mutex);
	
	// Write this from to the file
	if (_file.is_open())
	{
		if (frame.IsInitialized())
		{
			frame.SerializeToOstream(&_file);
		} else {
			vector<string> errors;
			frame.FindInitializationErrors(&errors);
			printf("Logger: Not writing frame missing fields:");
			BOOST_FOREACH(const string &str, errors)
			{
				printf(" %s", str.c_str());
			}
			printf("\n");
		}
	}
	
	// Get the place in the circular buffer where we will store this frame
	int i = _nextFrameNumber % _history.size();
	
	if (_history[i])
	{
		// Remove the space used by the old data
		_spaceUsed -= _history[i]->SpaceUsed();
	} else {
		// Create a new LogFrame
		_history[i] = new LogFrame();
	}
	
	// Store the frame
	_history[i]->CopyFrom(frame);
	
	// Add space used by the new data
	_spaceUsed += _history[i]->SpaceUsed();
	
	// Go to the next frame
	++_nextFrameNumber;
}

bool Logger::getFrame(int i, LogFrame& frame)
{
	QMutexLocker locker(&_mutex);
	
	if (i < 0 || i < (_nextFrameNumber - (int)_history.size()) || i >= _nextFrameNumber)
	{
		// Frame number is out of range
		return false;
	}
	
	frame.CopyFrom(*_history[i % _history.size()]);
	
	return true;
}

int Logger::getFrames(int start, vector<LogFrame> &frames)
{
	QMutexLocker locker(&_mutex);
	
	int minFrame = _nextFrameNumber - (int)_history.size();
	if (minFrame < 0)
	{
		minFrame = 0;
	}
	
	if (start < minFrame || start >= _nextFrameNumber)
	{
		return 0;
	}
	
	int end = start - frames.size() + 1;
	if (end < minFrame)
	{
		end = minFrame;
	}
	
	int n = start - end + 1;
	for (int i = 0; i < n; ++i)
	{
		frames[i].CopyFrom(*_history[(start - i) % _history.size()]);
	}
	
	for (int i = n; i < (int)frames.size(); ++i)
	{
		frames[i].Clear();
	}
	
	return n;
}
