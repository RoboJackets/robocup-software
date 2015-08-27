#include "Logger.hpp"

#include <QString>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;
using namespace Packet;
using namespace google::protobuf::io;

Logger::Logger() {
    _fd = -1;
    _history.resize(100000);
    _nextFrameNumber = 0;
    _spaceUsed = sizeof(_history[0]) * _history.size();
}

Logger::~Logger() { close(); }

bool Logger::open(QString filename) {
    QMutexLocker locker(&_mutex);

    if (_fd >= 0) {
        close();
    }

    _fd = creat(filename.toLatin1(), 0666);
    if (_fd < 0) {
        printf("Can't create %s: %m\n", (const char*)filename.toLatin1());
        return false;
    }

    _filename = filename;

    return true;
}

void Logger::close() {
    QMutexLocker locker(&_mutex);
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
        _filename = QString();
    }
}

void Logger::addFrame(shared_ptr<LogFrame> frame) {
    QMutexLocker locker(&_mutex);

    // Write this from to the file
    if (_fd >= 0) {
        if (frame->IsInitialized()) {
            uint32_t size = frame->ByteSize();
            if (write(_fd, &size, sizeof(size)) != sizeof(size)) {
                printf("Logger: Failed to write size, closing log: %m\n");
                close();
            } else if (!frame->SerializeToFileDescriptor(_fd)) {
                printf("Logger: Failed to write frame, closing log: %m\n");
                close();
            }
        } else {
            printf("Logger: Not writing frame missing fields: %s\n",
                   frame->InitializationErrorString().c_str());
        }
    }

    // Get the place in the circular buffer where we will store this frame
    int i = _nextFrameNumber % _history.size();

    if (_history[i]) {
        // Remove the space used by the old data
        _spaceUsed -= _history[i]->SpaceUsed();
    }

    // Create a new LogFrame
    _history[i] = frame;

    // Add space used by the new data
    _spaceUsed += _history[i]->SpaceUsed();

    // Go to the next frame
    ++_nextFrameNumber;
}

shared_ptr<LogFrame> Logger::lastFrame() const {
    QMutexLocker locker(&_mutex);
    return _history[(_nextFrameNumber - 1) % _history.size()];
}

int Logger::getFrames(int start, vector<shared_ptr<LogFrame> >& frames) const {
    QMutexLocker locker(&_mutex);

    int minFrame = _nextFrameNumber - (int)_history.size();
    if (minFrame < 0) {
        minFrame = 0;
    }

    if (start < minFrame || start >= _nextFrameNumber) {
        return 0;
    }

    int end = start - frames.size() + 1;
    if (end < minFrame) {
        end = minFrame;
    }

    int n = start - end + 1;
    for (int i = 0; i < n; ++i) {
        frames[i] = _history[(start - i) % _history.size()];
    }

    for (int i = n; i < (int)frames.size(); ++i) {
        frames[i].reset();
    }

    return n;
}
