#include "RobotStatus.hpp"

#include <QProgressBar>
#include <QGridLayout>
#include <QLabel>
#include <QCheckBox>

using namespace Log;

Log::RobotStatusType RobotStatusType::RobotStatus::_type;

QWidget* RobotStatusType::information()
{
	QWidget* infoWidget = new QWidget();
	QGridLayout* l = new QGridLayout(infoWidget);
	
	QString headers[] = {"Batt", "", "RSSI", "Charged", "Ball"};
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		QLabel* label = new QLabel(headers[i]);
		label->setAlignment(Qt::AlignCenter);
		l->addWidget(label, 0, i+1);
	}
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_batteries[i] = new QProgressBar();
		_batteries[i]->setRange(0, 150);
		_batteries[i]->setValue(0);
		_batteries[i]->setTextVisible(false);
		_batteries[i]->setFixedHeight(20);
		
		_voltages[i] = new QLabel("0 V");
		_voltages[i]->setFixedWidth(50);
		_voltages[i]->setAlignment(Qt::AlignRight);
		
		_rssi[i] = new QProgressBar();
		_rssi[i]->setRange(0,255);
		_rssi[i]->setValue(0);
		_rssi[i]->setFixedHeight(20);
		
		_charged[i] = new QCheckBox();
		_charged[i]->setEnabled(false);
		
		_ball[i] = new QCheckBox();
		_ball[i]->setEnabled(false);
		
		int c = 0;
		l->addWidget(new QLabel("R." + QString::number(i)), i+1, c++);
		l->addWidget(_batteries[i], i+1, c++);
		l->addWidget(_voltages[i], i+1, c++, Qt::AlignRight);
		l->addWidget(_rssi[i], i+1, c++);
		l->addWidget(_charged[i], i+1, c++, Qt::AlignCenter);
		l->addWidget(_ball[i], i+1, c++, Qt::AlignCenter);
	}
	
	//l->setRowStretch(5, 1);
	
	return infoWidget;
}

void RobotStatusType::RobotStatus::updateInformationWidget()
{
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_type._batteries[i]->setValue(int(_data.robots[i].battery * 10));
		//_type._voltages[i]->setText(QString::sprintf("%2.2f V", _data.robots[i].battery));
		
		_type._voltages[i]->setText(QString("%1 V ").arg(_data.robots[i].battery, 4, 'f', 2));
		_type._rssi[i]->setValue(_data.robots[i].rssi);
		_type._charged[i]->setChecked(_data.robots[i].charged);
		_type._ball[i]->setChecked(_data.robots[i].ballPresent);
	}
}
