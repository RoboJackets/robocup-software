/**
 * @brief The Logger stores and saves the state of the game at each point in
 * time.
 *
 * @details
 * The log stores things such as robot and ball position and velocity as well as
 * debug information about the current play. See the LogFrame.proto file for a
 * full list of what is stored in the log.
 *
 * This logger implements a circular buffer for recent history and writes all
 * frames to disk.
 *
 * _history is a circular buffer.
 *
 * Consider a sequence number for each frame, where the first frame passed
 * to addFrame() has a sequence number of zero and the sequence number is one
 * greater for each subsequent frame.
 *
 * _nextFrameNumber is the sequence number of the next frame to be stored by
 * addFrame().
 *
 * lastFrame() returns the sequence number of the latest available frame.
 * It returns -1 if no frames have been stored.
 *
 * You can get a copy of a frame by passing its sequence number to getFrame().
 * If the frame is too old to be in the circular buffer (or the sequence number
 * is beyond the most recent available) then getFrame() will return false.
 *
 * Frames are allocated as they are first needed.  The size of the circular
 * buffer limits total memory usage.
 */

#pragma once

#include <protobuf/LogFrame.pb.h>

#include <QString>
#include <QReadLocker>
#include <QWriteLocker>
#include <QReadWriteLock>
#include <vector>
#include <algorithm>
#include <memory>
#include "time.hpp"
#include <boost/circular_buffer.hpp>

class Logger {
public:
    Logger(size_t logSize = 10000);
    ~Logger();

    bool open(QString filename);
    void close();

    // Returns the size of the circular buffer
    size_t capacity() const { return _history.capacity(); }

    // Returns the sequence number of the most recently added frame.
    // Returns -1 if no frames have been added.
    size_t size() const { return _history.size(); }

    std::shared_ptr<Packet::LogFrame> lastFrame() const;

    void addFrame(std::shared_ptr<Packet::LogFrame> frame);

    // Gets frames.size() frames starting at start and working backwards.
    // Clears any frames that couldn't be populated.
    // Returns the number of frames copied.
    int getFrames(int start,
                  std::vector<std::shared_ptr<Packet::LogFrame>>& frames) const;

    // Returns the amount of memory used by all LogFrames in the history.
    int spaceUsed() const { return _spaceUsed; }

    bool recording() const { return _fd >= 0; }

    QString filename() const { return _filename; }

    int firstFrameNumber() const {
        return currentFrameNumber() - _history.size() + 1;
    }

    int currentFrameNumber() const { return _nextFrameNumber - 1; }

    template <typename OutputIterator>
    int getFrames(int endIndex, int num, OutputIterator result) const {
        QReadLocker locker(&_lock);
        auto end = _history.rbegin();
        endIndex = std::min(_nextFrameNumber, endIndex);
        int numFromBack = _nextFrameNumber - endIndex;
        if (numFromBack >= _history.size()) {
            return 0;
        } else {
            std::advance(end, numFromBack);
            int startFrame = _nextFrameNumber - _history.size();
            int numToCopy = std::min(endIndex - 1 - startFrame, num);
            copy_n(end, numToCopy, result);
            return std::max(numToCopy, 0);
        }
    }

    RJ::Time startTime() const { return _startTime; }

private:
    RJ::Time _startTime;
    mutable QReadWriteLock _lock;

    QString _filename;

    /**
     * Frame history.
     * Increasing indices correspond to earlier times.
     * This must only be accessed while _mutex is locked.
     *
     * It is not safe to modify a single std::shared_ptr from multiple threads,
     * but after it is copied the copies can be used and destroyed freely in
     * different threads.
     */
    boost::circular_buffer<std::shared_ptr<Packet::LogFrame>> _history;

    int _spaceUsed;

    // File descriptor for log file
    int _fd;

    // Sequence number of the next frame to be written
    int _nextFrameNumber = 0;
};
