#pragma once

#include <LogFrame.hpp>
#include <QTimer>

namespace Log
{
	class LogFile;
	
	/** acquires the frames */
	class LogControl : public QObject
	{
		Q_OBJECT;
		
		public:
			LogControl();
		
			void setLogFile(LogFile* logfile);
			
		private Q_SLOTS:
			/** fetch a new frame from the file */
			void fetchFrame();
				
		Q_SIGNALS:
			void newFrame(Packet::LogFrame* frame);
		
		private:
			/** the log file to control */
			LogFile* _logFile;
			
			/** last log frame */
			Packet::LogFrame _frame;

			/** timestamp of last frame */
			uint64_t _lastTimestamp;
			
			QTimer _fetchTimer;
	};
}
