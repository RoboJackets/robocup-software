//  put "#pragma once" at the top of header files to protect against being included multiple times
#pragma once
#include <QFont>
#include <QtWidgets>
#include <QPainter>
#include <QRect>

class BatteryWidget : public QWidget {
public:
int value;
	BatteryWidget ( QWidget * parent = 0, Qt::WindowFlags f = 0 ):QWidget(parent, f) {};
	virtual void paintEvent ( QPaintEvent * event )	{
		QRect rect = event->rect();			    
	    QPainter painter(this);
	    painter.setPen(Qt::black);
		QFont serifFont("Times", rect.height()*0.9, QFont::Bold);
		painter.setFont(serifFont);
	    painter.drawRect(rect.x(),rect.y(),rect.width()*0.9,rect.height());
	    painter.fillRect(rect.x(),rect.y(),(rect.width()*value*0.9/100 ),rect.height(),Qt::blue);// rectangle proportionnal to battery
	    int x= rect.x()+(rect.width());
	    int y =rect.y()+rect.height()/2 -5;
	    painter.drawRect(x,y,7,7);
	    painter.drawText(rect,Qt::AlignCenter,QString("%1").arg(value));
	}	
};
