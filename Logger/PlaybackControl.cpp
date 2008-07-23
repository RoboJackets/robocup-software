#include "PlaybackControl.hpp"

#include <QHBoxLayout>

PlaybackControl::PlaybackControl()
{
	_loggable = 0;
	_pause = false;
	
	QHBoxLayout* layout = new QHBoxLayout(this);
			
	_playPause = new QPushButton("||");
	_playPause->setCheckable(true);
	layout->addWidget(_playPause);
	connect(_playPause, SIGNAL(toggled(bool)), SLOT(playPause(bool)));
	//_playPause->setChecked(true);
	
	_timeSlider = new QSlider(Qt::Horizontal);
	layout->addWidget(_timeSlider);
	connect(_timeSlider, SIGNAL(sliderMoved(int)), SLOT(seek(int)));
	
	_timeSlider->setTickPosition(QSlider::TicksBothSides);
	_timeSlider->setMinimum(0);
	
	_timeLabel = new QLabel("0 s");
	layout->addWidget(_timeLabel);
	
	_span = new QDoubleSpinBox();
	layout->addWidget(_span);
	
	connect(_span, SIGNAL(valueChanged(double)), SLOT(setSpan(double)));
	_span->setValue(0.0);
	_span->setSingleStep(.01);
}

PlaybackControl::~PlaybackControl()
{
	
}

void PlaybackControl::resizeEvent(QResizeEvent* re)
{
	_timeSlider->setMaximum(_timeSlider->width());
	re->accept();
}

void PlaybackControl::setSpan(double span)
{
	if (_loggable)
	{
		_loggable->span(uint64_t(span * 1000000));
	}
}

void PlaybackControl::playPause(bool val)
{
	_pause = val;
	if (_loggable)
	{
		_loggable->pause(_pause);
	}
}

void PlaybackControl::seek(int val)
{
	if (_pause && _loggable)
	{
		//time/step
		const uint64_t startTime = _loggable->startTime();
		const uint64_t endTime = _loggable->latestReceivedTime();
		const uint64_t range = endTime - startTime;
		
		uint64_t time = range/_timeSlider->maximum() * val;
		
		_loggable->seek(time);
	}
}

void PlaybackControl::update()
{
	if (!_loggable)
	{
		return;
	}
	
	const uint64_t startTime = _loggable->startTime();
	const uint64_t endTime = _loggable->latestReceivedTime();
	const uint64_t lastTime = _loggable->latestDisplayTime();
	
	//update the label
	const uint64_t range = endTime - startTime;
	const float total = range/1000000.0;
	const float pos = (lastTime - startTime)/1000000.0;
	
	_timeLabel->setText(QString::number(pos, 'f', 2) + tr("/") + 
			QString::number(total, 'f', 2));
		
	const float per = (lastTime - startTime)/(double)range;
	const int px = int(per * _timeSlider->width());
	
	if (!_timeSlider->isSliderDown())
	{
		_timeSlider->setValue(px);
	}
}

