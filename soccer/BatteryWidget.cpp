#include "BatteryWidget.hpp"
#include <cmath>

BatteryWidget::BatteryWidget(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f) {
    _batteryLevel = 0;
    setBatteryLevel(0.5);
}

float BatteryWidget::batteryLevel() const { return _batteryLevel; }

void BatteryWidget::setBatteryLevel(float batteryLevel) {
    if (fabs(batteryLevel - _batteryLevel) > 0.01) {
        _batteryLevel = batteryLevel;
        if (_batteryLevel > 1) _batteryLevel = 1;
        if (_batteryLevel < 0) _batteryLevel = 0;

        //  trigger a redraw
        update();
    }
}

void BatteryWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QColor color = _batteryLevel > 0.25 ? Qt::black : Qt::red;

    painter.setPen(QPen(color, 2.5));

    float minPadding = 2;
    float h2w = 2;
    float w = fmin(width() - minPadding * 2, (height() - minPadding * 2) * h2w);
    float h = w / h2w;
    QRectF battBounds((width() - w) / 2, (height() - h) / 2, w, h);

    //  how big the positive terminal is relative to the main part of the
    //  battery
    float nubRatio = 0.14;

    //  draw main part of battery
    painter.setBrush(Qt::NoBrush);
    QRectF mainBox =
        battBounds.adjusted(0, 0, -battBounds.width() * nubRatio, 0);
    painter.drawRoundedRect(mainBox, 1, 1);

    painter.setBrush(color);

    //  nub (positive terminal of battery)
    QRectF nubBox = battBounds.adjusted(battBounds.width() * (1 - nubRatio),
                                        battBounds.height() / 4, 0,
                                        -battBounds.height() / 4);
    painter.drawRoundedRect(nubBox, 0.5, 0.5);

    //  12.5%, 37.5%, 62.5%, 87.5% are the midpoints of the 4 bars
    //  we draw the number of bars corresponding to the point that this battery
    //  level is closest to
    int barCount = ceil(_batteryLevel * 4 - 0.24);

    //  draw bars to show battery level
    float pad = 3;
    float barSpacing = 2;
    float barWidth = (mainBox.width() - pad * 2 - barSpacing * 3) / 4.0;
    painter.setPen(Qt::NoPen);
    for (int i = 0; i < barCount; i++) {
        painter.drawRect(
            QRectF(mainBox.left() + pad + (barSpacing + barWidth) * i,
                   mainBox.top() + pad, barWidth, mainBox.height() - pad * 2));
    }
}
