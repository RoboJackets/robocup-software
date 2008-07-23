#include "CommData.hpp"

#include <QWidget>
#include <QProgressBar>
#include <QGridLayout>
#include <QLabel>

using namespace Log;

Log::CommDataType CommDataType::CommData::_type;

QWidget* CommDataType::information()
{
	QWidget* infoWidget = new QWidget();
	QGridLayout* l = new QGridLayout(infoWidget);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		for (unsigned int j=0 ; j<4 ; ++j)
		{
			_motors[i][j] = new QProgressBar();
			_motors[i][j]->setMinimum(-128);
			_motors[i][j]->setMaximum(127);
			_motors[i][j]->setValue(0);
			_motors[i][j]->setFormat("%v");
			_motors[i][j]->setFixedHeight(20);
			
			l->addWidget(new QLabel("R."+QString::number(i)), i, 0);
			l->addWidget(_motors[i][j], i, j+1);
		}
	}
	
	l->setRowStretch(5, 1);
	
	return infoWidget;
}

void CommDataType::CommData::updateInformationWidget()
{
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		if (_data.robots[i].valid)
		{
			_type._motors[i][0]->setValue(_data.robots[i].motor[0]);
			_type._motors[i][1]->setValue(_data.robots[i].motor[1]);
			_type._motors[i][2]->setValue(_data.robots[i].motor[2]);
			_type._motors[i][3]->setValue(_data.robots[i].motor[3]);
		}
	}
}
