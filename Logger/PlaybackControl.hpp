#ifndef PLAYBACKCONTROL_HPP_
#define PLAYBACKCONTROL_HPP_

#include "Log/Loggable.hpp"

#include <QWidget>
#include <QResizeEvent>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>

class PlaybackControl : public QWidget
{
	Q_OBJECT;
	
	public:
		PlaybackControl();
		~PlaybackControl();
		
		/** set the loggable to operate on */
		void loggable(Log::Loggable* loggable)
		{
			_loggable = loggable;
			loggable->pause(_pause);
		}
		
		void update();
	
	protected Q_SLOTS:
		void setSpan(double span);
		void playPause(bool val);
		void seek(int);
		
	protected:
		void resizeEvent(QResizeEvent* re);
		
	private:
		Log::Loggable* _loggable;
		
		QPushButton* _playPause;
		QSlider* _timeSlider;
		QLabel* _timeLabel;
		QDoubleSpinBox* _span;
		
		bool _pause;
};

#endif /*PLAYBACKCONTROL_HPP_*/
