#pragma once

#include <rj_protos/LogFrame.pb.h>

#include <QColor>
#include <QPointF>

[[maybe_unused]] static inline QColor qcolor(uint32_t value) {
    uint8_t a = value >> 24; //NOLINT(readability-identifier-length)
    uint8_t r = value >> 16; //NOLINT(readability-identifier-length)
    uint8_t g = value >> 8; //NOLINT(readability-identifier-length)
    uint8_t b = value; //NOLINT(readability-identifier-length)
    return {r, g, b, a};
}

[[maybe_unused]] static inline QPointF qpointf(const Packet::Point& pt) { //NOLINT(readability-identifier-length)
    return {pt.x(), pt.y()};
}

[[maybe_unused]] static inline uint32_t color(const QColor& color) {
    return (color.red() << 16) | (color.green() << 8) | (color.blue()) | (color.alpha() << 24);
}
