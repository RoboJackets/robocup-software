#pragma once

#include <QColor>
#include <protobuf/LogFrame.pb.h>

static inline QColor qcolor(uint32_t value) {
    uint8_t r = value >> 16;
    uint8_t g = value >> 8;
    uint8_t b = value;
    return QColor(r, g, b);
}

static inline QPointF qpointf(const Packet::Point& pt) {
    return QPointF(pt.x(), pt.y());
}

static inline uint32_t color(const QColor& color) {
    return (color.red() << 16) | (color.green() << 8) | (color.blue());
}
