#include "Logger.hpp"

#include <QString>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "Utils.hpp"

using namespace std;
using namespace Packet;
using namespace google::protobuf::io;

Logger::Logger(size_t logSize) : _history(logSize) {
    _fd = -1;
    _spaceUsed = sizeof(shared_ptr<Packet::LogFrame>) * _history.size();
}

Logger::~Logger() { close(); }

bool Logger::open(QString filename) {
    QWriteLocker locker(&_lock);

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
    QWriteLocker locker(&_lock);
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
        _filename = QString();
    }
}

void Logger::addFrame(shared_ptr<LogFrame> frame) {
    this->addFrame(frame, false);
}

void Logger::addFrame(shared_ptr<LogFrame> frame, bool force) {
    QWriteLocker locker(&_lock);

    if (_history.empty()) {
        _startTime = RJ::Time(chrono::microseconds(frame->timestamp()));
    }

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

    if (_history.full() && !force) {
        _spaceUsed -= _history.front()->SpaceUsed();
        _history.pop_front();
    }
    // Create a new LogFrame

    _history.push_back(frame);

    // Add space used by the new data
    _spaceUsed += frame->SpaceUsed();
    _nextFrameNumber++;
}

shared_ptr<LogFrame> Logger::lastFrame() const {
    QReadLocker locker(&_lock);
    return _history.back();
}

void Logger::clear() {
    QReadLocker locker(&_lock);
    _history.clear();
    _spaceUsed = 0;
}

// Clears out existing logs
bool Logger::readFrames(const char* filename) {
    this->clear();

    QFile file(filename);
    if (!file.open(QFile::ReadOnly)) {
        fprintf(stderr, "Can't open %s: %s\n", filename,
                (const char*)file.errorString().toLatin1());
        return false;
    }

    int n = 0;
    while (!file.atEnd()) {
        uint32_t size = 0;
        if (file.read((char*)&size, sizeof(size)) != sizeof(size)) {
            // Broken length
            printf("Broken length\n");
            return false;
        }

        string str(size, 0);
        if (file.read(&str[0], size) != size) {
            // Broken packet at end of file
            printf("Broken packet\n");
            return false;
        }

        std::shared_ptr<LogFrame> frame = std::make_shared<LogFrame>();
        if (!frame->ParsePartialFromString(str)) {
            printf("Failed: %s\n", frame->InitializationErrorString().c_str());
            return false;
        }
        this->addFrame(frame, true);
        ++n;
    }

    return true;
}
