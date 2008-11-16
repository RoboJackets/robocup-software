#ifndef LOGCONTROL_HPP_
#define LOGCONTROL_HPP_

#include <QWidget>
#include <LogFrame.hpp>
#include <QTimer>

class QPushButton;

namespace Log
{
	class JogDial;
	class LogFile;
	
	/** widget to control playback of log file */
	class LogControl : public QWidget
	{
		Q_OBJECT;
		
		public:
			LogControl(QWidget* parent = 0);
		
			void setLogFile(LogFile* logfile);
			
			void live(bool live);
			
		private Q_SLOTS:
			/** fetch a new frame from the file */
			void fetchFrame();
			
			void on_pause_clicked();
			void on_seekStart_clicked();
			void on_seekEnd_clicked();
			void on_dial_valueChanged(float);
				
		Q_SIGNALS:
			void newFrame(Packet::LogFrame* frame);
		
		private:
			JogDial* _dial;
			QPushButton* _pause;
			QPushButton* _seekEnd;
			QPushButton* _seekStart;
			
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

#endif /* LOGCONTROL_HPP_ */
