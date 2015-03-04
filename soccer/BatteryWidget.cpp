#include "BatteryWidget.hpp"


BatteryWidget::BatteryWidget(QWidget *parent, Qt::WindowFlags f) : QWidget(parent, flags) {
    _batteryLevel = 0;
    setBatteryLevel(0.5);
}

float BatteryWidget::batteryLevel() const {
    return _batteryLevel;
}

void BatteryWidget::setBatteryLevel(float batteryLevel) {
    if (abs(batteryLevel - _batteryLevel) > 0.01) {
        _batteryLevel = batteryLevel;

        //  trigger a redraw
        update();
    }
}

void BatteryWidget::paintEvent(QPaintEvent *event) {
    QPainter painter(this);

    //  FIXME: set antialiasing on

    painter.setPen(Qt::black);

    painter.setFont(QFont("Times", rect.height()*0.9, QFont::Bold));

    QRect rect = event->rect();
    painter.drawRect(rect.x(),rect.y(),rect.width()*0.9,rect.height());
    painter.fillRect(rect.x(),rect.y(),(rect.width()*value*0.9/100 ),rect.height(),Qt::blue);   // rectangle proportionnal to battery
    int x = rect.x() + (rect.width());
    int y = rect.y() + rect.height()/2 -5;
    painter.drawRect(x, y, 7, 7);
    painter.drawText(rect,Qt::AlignCenter,QString("%1").arg(value));
}
