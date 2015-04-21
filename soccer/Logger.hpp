/**
 * @brief The Logger stores and saves the state of the game at each point in time.
 * 
 * @details
 * This logger implements a circular buffer for recent history and writes all
 * frames to disk.
 *
 * _history is a circular buffer.
 *
 * Consider a sequence number for each frame, where the first frame passed
 * to addFrame() has a sequence number of zero and the sequence number is one greater for
 * each subsequent frame.
 *
 * _nextFrameNumber is the sequence number of the next frame to be stored by addFrame().
 *
 * lastFrame() returns the sequence number of the latest available frame.
 * It returns -1 if no frames have been stored.
 *
 * You can get a copy of a frame by passing its sequence number to getFrame().
 * If the frame is too old to be in the circular buffer (or the sequence number is
 * beyond the most recent available) then getFrame() will return false.
 *
 * Frames are allocated as they are first needed.  The size of the circular buffer
 * limits total memory usage.
 */

#pragma once

#include <protobuf/LogFrame.pb.h>

#include <QString>
#include <QMutexLocker>
#include <QMutex>
#include <vector>
#include <algorithm>
#include <memory>

class Logger
{
	public:
		Logger();
		~Logger();
		
		bool open(QString filename);
		void close();
		
		// Returns the number of available frames
		int numFrames() const
		{
			QMutexLocker locker(&_mutex);
			return std::min(_nextFrameNumber, (int)_history.size());
		}
		
		// Returns the size of the circular buffer
		int maxFrames() const
		{
			QMutexLocker locker(&_mutex);
			return _history.size();
		}
		
		// Returns the sequence number of the earliest available frame.
		// Returns -1 if no frames have been added.
		int firstFrameNumber() const
		{
			QMutexLocker locker(&_mutex);
			if (_nextFrameNumber == 0)
			{
				return -1;
			} else {
				return std::max(0, _nextFrameNumber - (int)_history.size());
			}
		}
		
		// Returns the sequence number of the most recently added frame.
		// Returns -1 if no frames have been added.
		int lastFrameNumber() const
		{
			QMutexLocker locker(&_mutex);
			return _nextFrameNumber - 1;
		}
		
		std::shared_ptr<Packet::LogFrame> lastFrame() const;
		
		void addFrame(std::shared_ptr<Packet::LogFrame> frame);
		
		// Gets frames.size() frames starting at <i> and working backwards.
		// Clears any frames that couldn't be populated.
		// Returns the number of frames copied.
		int getFrames(int start, std::vector<std::shared_ptr<Packet::LogFrame> > &frames) const;
		
		// Returns the amount of memory used by all LogFrames in the history.
		int spaceUsed() const
		{
			QMutexLocker locker(&_mutex);
			return _spaceUsed;
		}
		
		bool recording() const
		{
			QMutexLocker locker(&_mutex);
			return _fd >= 0;
		}
		
		QString filename() const
		{
			QMutexLocker locker(&_mutex);
			return _filename;
		}
		
	private:
		mutable QMutex _mutex;
		
		QString _filename;
		
		/**
		 * Frame history.
		 * Increasing indices correspond to earlier times.
		 * This must only be accessed while _mutex is locked.
		 *
		 * It is not safe to modify a single std::shared_ptr from multiple threads,
		 * but after it is copied the copies can be used and destroyed freely in different threads.
		 */
		std::vector<std::shared_ptr<Packet::LogFrame> > _history;
		
		// Sequence number of the next frame to be written
		int _nextFrameNumber;
		
		int _spaceUsed;
		
		// File descriptor for log file
		int _fd;
};
