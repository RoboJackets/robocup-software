#include "LogControl.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QFrame>
#include <QPushButton>
#include <QTimer>

#include "LogFile.hpp"

using namespace Log;

LogControl::LogControl() : // QWidget* parent) :
//	QWidget(parent),
	_logFile(0), _lastTimestamp(0)
{
	_msecMult = 1;
	_live = true;
	
	_fetchTimer.setSingleShot(true);
	connect(&_fetchTimer, SIGNAL(timeout()), SLOT(fetchFrame()));
}

void LogControl::setLogFile(LogFile* logfile)
{
	_logFile = logfile;
	
	_lastTimestamp = 0;
	fetchFrame();
}

void LogControl::live(bool live)
{
	_live = live;
}


void LogControl::fetchFrame()
{
	if (_logFile)
	{
		//first frame fetching after setting new logfile
		if (_lastTimestamp == 0)
		{
			if (_logFile->hasNextFrame())
			{
				_frame = _logFile->readNext();
				_lastTimestamp = _frame.timestamp;
				
				//immediately call because signal happens after timestamp is set
				fetchFrame();
                return;
			}
			
            //wait and try again after some time (.5 sec)
            QTimer::singleShot(500, this, SLOT(fetchFrame()));
			return;
		}
        
		newFrame(&_frame);
		
		if (_live)
		{
			_frame = _logFile->readLast();
			_fetchTimer.start(34);
            
            return;
		}
		else if (_logFile->hasNextFrame())
		{	
			_fetchTimer.stop();
			_frame = _logFile->readNext();
		}
		
		if (!_live)
		{
			int64_t diff = _frame.timestamp - _lastTimestamp;
			int msec = abs((int)(_msecMult * diff / 1000));
			_lastTimestamp = _frame.timestamp;
			
			_fetchTimer.start(msec);
		}
	}
}
