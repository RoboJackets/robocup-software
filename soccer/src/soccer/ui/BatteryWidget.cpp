#include "BatteryWidget.hpp"

#include <cmath>

BatteryWidget::BatteryWidget(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f) {
    _batteryLevel = 0;
    setBatteryLevel(0.5);
}

float BatteryWidget::batteryLevel() const { return _batteryLevel; }

void BatteryWidget::setBatteryLevel(float battery_level) {
    if (std::fabs(battery_level - _batteryLevel) > 0.01) {
        _batteryLevel = battery_level;
        if (_batteryLevel > 1) {
            _batteryLevel = 1;
        }
        if (_batteryLevel < 0) {
            _batteryLevel = 0;
        }

        //  trigger a redraw
        update();
    }
}

void BatteryWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QColor color = _batteryLevel > 0.25 ? Qt::black : Qt::red;

    painter.setPen(QPen(color, 2.5));

    float min_padding = 2;
    float h2w = 2;
    float w = std::fmin(width() - min_padding * 2,
                        (height() - min_padding * 2) * h2w);
    float h = w / h2w;
    QRectF batt_bounds((width() - w) / 2, (height() - h) / 2, w, h);

    //  how big the positive terminal is relative to the main part of the
    //  battery
    float nub_ratio = 0.14;

    //  draw main part of battery
    painter.setBrush(Qt::NoBrush);
    QRectF main_box =
        batt_bounds.adjusted(0, 0, -batt_bounds.width() * nub_ratio, 0);
    painter.drawRoundedRect(main_box, 1, 1);

    painter.setBrush(color);

    //  nub (positive terminal of battery)
    QRectF nub_box = batt_bounds.adjusted(batt_bounds.width() * (1 - nub_ratio),
                                          batt_bounds.height() / 4, 0,
                                          -batt_bounds.height() / 4);
    painter.drawRoundedRect(nub_box, 0.5, 0.5);

    //  12.5%, 37.5%, 62.5%, 87.5% are the midpoints of the 4 bars
    //  we draw the number of bars corresponding to the point that this battery
    //  level is closest to
    int bar_count = ceil(_batteryLevel * 4 - 0.24);

    //  draw bars to show battery level
    float pad = 3;
    float bar_spacing = 2;
    float bar_width = (main_box.width() - pad * 2 - bar_spacing * 3) / 4.0;
    painter.setPen(Qt::NoPen);
    for (int i = 0; i < bar_count; i++) {
        painter.drawRect(QRectF(
            main_box.left() + pad + (bar_spacing + bar_width) * i,
            main_box.top() + pad, bar_width, main_box.height() - pad * 2));
    }
}
