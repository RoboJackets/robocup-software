#ifndef LOGFILE_HPP_
#define LOGFILE_HPP_

#include <QString>
#include <QFile>
#include <QDataStream>
#include <QMutex>

#include <Serialization.hpp>

#include <LogFrame.hpp>

namespace Log
{
	class LogFile
	{
		/// types ///
		private:
			typedef uint16_t FrameSizeType;
			
		/// methods ///
		public:
			LogFile(QString filename);
			~LogFile();
			
			/** create a filename based on the current time */
			static QString genFilename();
			
			/** write the log frame to the end of the file */
			void write(Packet::LogFrame& frame);
			
			/** read the next frame after the last frame read */
			Packet::LogFrame readNext();
			
			/** read the previous frame of the last frame read */
			Packet::LogFrame readPrev();
			
			/** read the latest/last log frame */
			Packet::LogFrame readLast();
			
			/** reset reading to the start of the file */
			void reset();
			
			/** true if there is more to read going forward */
			bool hasNextFrame() const;
			bool hasPrevFrame() const;
		
		/// members ///
		private:
			mutable QMutex _fileMutex;
			
			QFile _file;
			QDataStream* _stream;
			
			/** position of the next frame to read */
			qint64 _nextPos;
			/** position of previous frame to read */
			qint64 _prevPos;
			
			/** buffer for ser/des data */
			Serialization::MemoryBuffer _buf;
	};
}

#endif /* LOGFILE_HPP_ */
