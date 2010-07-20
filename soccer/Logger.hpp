#pragma once

#include <protobuf/LogFrame.pb.h>

#include <QMutexLocker>
#include <QMutex>
#include <vector>

class Logger
{
	public:
		Logger();
		
		int numFrames() const
		{
			return _history.size();
		}
		
		void addFrame(const Packet::LogFrame &frame);
		
		// Returns the i-th oldest frame.
		// Frame 0 is the most recent frame.
		void getFrame(int i, Packet::LogFrame &frame)
		{
			QMutexLocker locker(&_mutex);
			// This is signed because we're going backwards in time
			int n = _history.size();
			frame = _history[(_nextPos - i - 1 + n) % n];
		}
		
	protected:
		QMutex _mutex;
		
		// Frame history.
		// Increasing indices correspond to earlier times.
		std::vector<Packet::LogFrame> _history;
		
		// Index of the next frame to be written
		int _nextPos;
};
