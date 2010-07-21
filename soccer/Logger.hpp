// This logger implements a circular buffer for recent history and writes all
// frames disk (FIXME - not yet).
//
// _history is a circular buffer.
//
// Consider a sequence number for each frame, where the first frame passed
// to addFrame() has a sequence number of zero and the sequence number is one greater for
// each subsequent frame.
//
// _nextFrameNumber is the sequence number of the next frame to be stored by addFrame().
//
// lastFrame() returns the sequence number of the latest available frame.
// It returns -1 if no frames have been stored.
//
// You can get a copy of a frame by passing its sequence number to getFrame().
// If the frame is too old to be in the circular buffer (or the sequence number is
// beyond the most recent available) then getFrame() will return false.

#pragma once

#include <protobuf/LogFrame.pb.h>

#include <QMutexLocker>
#include <QMutex>
#include <vector>
#include <algorithm>

class Logger
{
	public:
		Logger();
		
		// Returns the number of available frames
		int numFrames()
		{
			QMutexLocker locker(&_mutex);
			return std::min(_nextFrameNumber, (int)_history.size());
		}
		
		// Returns the sequence number of the most recently added frame.
		// This will be -1 if no frames have been added.
		int lastFrame()
		{
			QMutexLocker locker(&_mutex);
			return _nextFrameNumber - 1;
		}
		
		void addFrame(const Packet::LogFrame &frame);
		
		// Gets frame <i>, if available.
		// Returns true if the frame was available or false if not (too old).
		bool getFrame(int i, Packet::LogFrame &frame);
		
		// Returns the amount of memory used by all LogFrames in the history.
		int spaceUsed();
		
	protected:
		QMutex _mutex;
		
		// Frame history.
		// Increasing indices correspond to earlier times.
		std::vector<Packet::LogFrame> _history;
		
		// Sequence number of the next frame to be written
		int _nextFrameNumber;
};
