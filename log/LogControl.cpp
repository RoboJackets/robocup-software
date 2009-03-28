#include "LogControl.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QFrame>
#include <QPushButton>
#include <QTimer>

#include "LogFile.hpp"
#include "JogDial.hpp"

using namespace Log;

LogControl::LogControl(QWidget* parent) :
	QWidget(parent), _logFile(0), _lastTimestamp(0)
{
	QHBoxLayout* layout = new QHBoxLayout();
	this->setLayout(layout);
	
	_pause = new QPushButton("||");
	_pause->setObjectName("pause");
	_pause->setCheckable(true);
	layout->addWidget(_pause);
	
	_seekStart = new QPushButton("<<");
	_seekStart->setObjectName("seekStart");
	layout->addWidget(_seekStart);
	
	QFrame* frame = new QFrame();
	frame->setLayout(new QHBoxLayout());
	frame->setFrameStyle(QFrame::Sunken | QFrame::StyledPanel);
	frame->layout()->setContentsMargins(1,1,1,1);
	frame->setFixedSize(200, 25);
	
	_dial = new JogDial(frame);
	_dial->setObjectName("dial");
	frame->layout()->addWidget(_dial);
	
	layout->addWidget(frame);
	
	_seekEnd = new QPushButton(">>");
	_seekEnd->setObjectName("seekEnd");
	layout->addWidget(_seekEnd);
	
	QMetaObject::connectSlotsByName(this);
	
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

void LogControl::on_pause_clicked()
{
	_live = false;
	
	fetchFrame();
}

void LogControl::on_seekStart_clicked()
{
	if (_logFile)
	{
		_logFile->reset();
		
		_lastTimestamp = 0;
		
		if (_pause->isChecked())
		{
			fetchFrame();
		}
	}
}

void LogControl::on_seekEnd_clicked()
{
	if (_logFile)
	{
		if (!_live)
		{
			_fetchTimer.stop();
			
			//_frame = _logFile->readLast();
			//_lastTimestamp = _frame.vision.timestamp;
			
			_live = true;
			fetchFrame();
		}
	}
}

void LogControl::on_dial_valueChanged(float val)
{
	if (_pause->isChecked())
	{
		if (val == 0)
		{
			_msecMult = 1;
		}
		else
		{
			_msecMult = fabs(1.0f/(val * 2));
		}
	}
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
				
				//immediately call because signal happens at start
				fetchFrame();
			}
			else
			{
				//wait and try again after some time (.5 sec)
				QTimer::singleShot(500, this, SLOT(fetchFrame()));
			}
			
			return;
		}
		
		newFrame(&_frame);
		
		if (_pause->isChecked())
		{
			float off = _dial->offset();
			if (off < 0 && _logFile->hasPrevFrame())
			{
				//read previous
				_frame = _logFile->readPrev();
			}
			else if (off > 0 && _logFile->hasNextFrame())
			{
				//read next
				_frame = _logFile->readNext();
			}
			else
			{
				_fetchTimer.start(500);
				return;
			}
		}
		else if (_live)
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
