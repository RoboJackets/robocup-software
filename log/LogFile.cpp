#include "LogFile.hpp"

#include <QTime>
#include <QMutexLocker>
#include <Point.hpp>
#include <Segment.hpp>
#include <Polygon.hpp>

using namespace Log;

LogFile::LogFile(QString filename)
:	_file(filename)
{
	_file.open(QIODevice::ReadWrite);
	_stream = new QDataStream(&_file);
	
	_nextPos = _prevPos = 0;
}

LogFile::~LogFile()
{
	_file.close();

	delete _stream;
}

QString LogFile::genFilename()
{
	QDateTime time = QDateTime::currentDateTime();
	
	return time.toString("yyyyMMdd.hhmmss.log");
}

void LogFile::write(Packet::LogFrame& frame)
{
	QMutexLocker ml(&_fileMutex);
	
	//seek to end to write
	_file.seek(_file.size());

	_buf.data.resize(0);

	Serialization::WriteBuffer& writer = _buf;
	writer & frame;

	FrameSizeType size = _buf.data.size();

	*_stream << size;
	_stream->writeRawData((const char*)&_buf.data[0], _buf.data.size());
	*_stream << size;
}

void LogFile::setLast(Packet::LogFrame& frame) {
	_last = frame;
}

Packet::LogFrame LogFile::readNext()
{
	QMutexLocker ml(&_fileMutex);
	
	Packet::LogFrame frame;
	
	//printf("Prev: %lld \tNext: %lld\n", _prevPos, _nextPos);
	_prevPos = _nextPos - sizeof(FrameSizeType);
	
	//read from next position
	_file.seek(_nextPos);
	
	//read size
	FrameSizeType size = 0;
	*_stream >> size;
	
	//read frame
	_buf.data.resize(size);
	_buf.rewind();
	_stream->readRawData((char*)&_buf.data[0], size);

	Serialization::ReadBuffer &reader = _buf;
	reader & frame;
	
	_nextPos = _file.pos() + sizeof(FrameSizeType);
	
	//printf("Prev: %lld \tNext: %lld\n", _prevPos, _nextPos);
	
	return frame;
}


Packet::LogFrame LogFile::readPrev()
{
	QMutexLocker ml(&_fileMutex);
	
	Packet::LogFrame frame;
	
	//printf("Prev: %lld \tNext: %lld\n", _prevPos, _nextPos);
	
	_nextPos = _prevPos + sizeof(FrameSizeType);
	
	//read size for packet
	_file.seek(_prevPos);
	
	FrameSizeType size = 0;
	*_stream >> size;
	
	//seek to start of packet
	_file.seek(_prevPos - size);
	
	//read frame
	_buf.data.resize(size);
	_buf.rewind();
	_stream->readRawData((char*)&_buf.data[0], size);

	Serialization::ReadBuffer &reader = _buf;
	reader & frame;
	
	_prevPos -= size + 2 * sizeof(FrameSizeType);
	
	//printf("Prev: %lld \tNext: %lld\n", _prevPos, _nextPos);
	
	return frame;
}

Packet::LogFrame LogFile::readLast()
{
    QMutexLocker ml(&_fileMutex);
	return _last;
}

bool LogFile::hasNextFrame() const
{
	QMutexLocker ml(&_fileMutex);
	
	return (_nextPos < _file.size());
}

bool LogFile::hasPrevFrame() const
{
	return (_prevPos > 0);
}

void LogFile::reset()
{
	QMutexLocker ml(&_fileMutex);
	
	_file.seek(0);
	_prevPos = _nextPos = 0;
}
