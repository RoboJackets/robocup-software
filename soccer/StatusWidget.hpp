//  put "#pragma once" at the top of header files to protect against being included multiple times
#pragma once
#include "ui_StatusWidget.h"
#include <QtWidgets>
#include <QPainter>
#include <QRect>
#include <string>
#include <QString>
#include <string>
#include <QApplication>
#include <QHBoxLayout>
#include <QObject>
#include <QtSvg>
#include <QImage>
#include <QSvgRenderer>
// these functions are called in mainWindow.cpp
class StatusWidget : public QWidget {
public:

	Ui_StatusWidget _ui;	

	StatusWidget ( QWidget * parent = 0, Qt::WindowFlags f = 0 ):QWidget(parent, f) 
	{
   		_ui.setupUi(this);
   		_ui.wifi->load(QString("../radio-connected.svg"));
	};

	virtual void paintEvent ( QPaintEvent * event )	{
	    QPainter painter(this);
	    painter.setRenderHint(QPainter::Antialiasing);
	    painter.setPen(Qt::black);	
	}
	void set_signal(bool signal) 
	{ 
		if(signal==true) {_ui.wifi->load(QString("../radio-connected.svg"));}
		else{_ui.wifi->load(QString("../radio-disconnected.svg"));}	
	}

	void set_image(bool image)
	{
	if (image==true){_ui.image->load(QString("../vision-available.svg"));}
	else {_ui.image->load(QString("../vision-unavailable.svg"));}
	}
	void set_battery(int value){_ui.battery->value=value;}
	void set_id(int id){_ui.Id->setText(QString("%1").arg(id));}
	void set_serial(std::string s){_ui.serial->setText(QString::fromStdString(s));}//
};
