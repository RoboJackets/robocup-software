#include "Ref.hpp"

#include <QWidget>
#include <QGridLayout>
#include <QLabel>

using namespace Log;

Log::RefType RefType::Ref::_type;

QWidget* RefType::information()
{
	QWidget* info = new QWidget();
	QGridLayout* layout = new QGridLayout(info);
	
	_state = new QLabel();
	layout->addWidget(new QLabel("<strong>State: </strong>"), 0, 0);
	layout->addWidget(_state, 0, 1);
	
	_period = new QLabel();
	layout->addWidget(new QLabel("<strong>Period: </strong>"), 0, 2);
	layout->addWidget(_period, 0, 3);

	_start = new QLabel();
	layout->addWidget(new QLabel("<strong>Start: </strong>"), 0, 4);
	layout->addWidget(_start, 0, 5);
		
	_ourStart = new QLabel();
	layout->addWidget(new QLabel("<strong>Our Start: </strong>"), 0, 6);
	layout->addWidget(_ourStart, 0, 7);
	
	return info;
}

void RefType::Ref::updateInformationWidget()
{
	QString state;
	QString stateColor;
	switch (_data.state)
	{
		case Packet::Ref::Halt:
			state = "Halt";
			stateColor = "red";
			break;
		case Packet::Ref::Stop:
			state = "Stop";
			stateColor = "pink";
			break;
		case Packet::Ref::Setup:
			state = "Setup";
			stateColor = "orange";
			break;
		case Packet::Ref::OppStart:
			state = "Opp Start";
			stateColor = "blue";
			break;
		case Packet::Ref::Running:
			state = "Running";
			stateColor = "green";
			break;
	}
	
	_type._state->setText(state);
	_type._state->setStyleSheet("color: " + stateColor);
	
	QString start;
	switch (_data.start)
	{
		case Packet::Ref::None:
			start = "None";
			break;
		case Packet::Ref::Kickoff:
			start = "Kickoff";
			break;
		case Packet::Ref::Penalty:
			start = "Penalty";
			break;
		case Packet::Ref::Direct:
			start = "Direct";
			break;
		case Packet::Ref::Indirect:
			start = "Indirect";
			break;
	}
	
	_type._start->setText(start);
	
	_type._ourStart->setText(_data.ourStart ? "true" : "false");
	if (_data.ourStart)
	{
		_type._ourStart->setStyleSheet("color: green");
	}
	else
	{
		_type._ourStart->setStyleSheet("color: red");
	}
	
	QString period;
	switch (_data.period)
	{
		case Packet::Ref::FirstHalf:
			period = "first";
			break;
		case Packet::Ref::HalfTime:
			period = "half";
			break;
		case Packet::Ref::SecondHalf:
			period = "second";
			break;
		case Packet::Ref::Overtime1:
			period = "overtime 1";
			break;
		case Packet::Ref::Overtime2:
			period = "overtime 2";
			break;
		case Packet::Ref::PenaltyShootout:
			period = "penalty";
			break;
	}
	
	_type._period->setText(period);
}
