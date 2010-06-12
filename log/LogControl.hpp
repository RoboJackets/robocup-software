#pragma once

//#include <QWidget>
#include <QObject>
#include <LogFrame.hpp>
#include <QTimer>

class QPushButton;

namespace Log
{
	class LogFile;
	
	/** widget to control playback of log file */
	class LogControl : public QObject // : public QWidget
	{
		Q_OBJECT;
		
		public:
			LogControl(); //QWidget* parent = 0);
		
			void setLogFile(LogFile* logfile);
			
			void live(bool live);
			
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
			
			float _msecMult;
			
			QTimer _fetchTimer;
			
			bool _live;
	};
}
