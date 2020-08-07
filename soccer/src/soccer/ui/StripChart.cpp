#include "StripChart.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <QDateTime>
#include <QFileDialog>
#include <QPainter>
#include <google/protobuf/descriptor.h>

#include <Geometry2d/Point.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_protos/LogFrame.pb.h>

using namespace std;
using namespace Packet;
using namespace boost;
using namespace google::protobuf;

StripChart::StripChart(QWidget* /*parent*/) {
    _history = nullptr;
    _minValue = 0;
    _maxValue = 1;
    //_function = 0;
    autoRange = true;
    _color = Qt::yellow;

    QPalette p = palette();
    p.setColor(QPalette::Window, Qt::black);
    setPalette(p);
    setAutoFillBackground(true);

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(100, 100);

    setMouseTracking(true);
}

void StripChart::function(Chart::Function* function) {
    if (function != nullptr) {
        _functions.append(function);
    }
}

void StripChart::exportChart() {
    QString chartName = QFileDialog::getSaveFileName(this, tr("Save Chart"), "run/newChart.csv",
                                                     tr("Csv Files(*.csv)"));
    std::ofstream outfile(chartName.toStdString());

    // output column names
    outfile << "Time";
    for (auto* function : _functions) {
        outfile << ", " << function->name.toStdString();
    }
    outfile << std::endl;

    // output data

    // Get the oldest datapoint to use as the starting time
    auto startTime = _history->at(0).get()->timestamp();

    for (const auto& frame_i : *_history) {
        if (frame_i) {
            outfile << RJ::timestamp_to_secs(frame_i->timestamp() - startTime);

            for (auto* function : _functions) {
                float v = 0;

                if (function->value(*frame_i, &v)) {
                    outfile << "," << v;
                }
            }
            outfile << std::endl;
        }
    }
    outfile.close();
}

QPointF StripChart::dataPoint(int i, float value) {
    auto x = static_cast<float>(i * width()) / static_cast<float>(chartSize);
    auto h = static_cast<float>(height());
    float y = h - (value - _minValue) * h / (_maxValue - _minValue);
    return QPointF(x, y);
}

int StripChart::indexAtPoint(const QPoint& point) {
    return chartSize - 1 - (width() - point.x()) * chartSize / width();
}

void StripChart::paintEvent(QPaintEvent* /*e*/) {
    if (_history == nullptr || _history->empty() || _functions.isEmpty()) {
        return;
    }

    QPainter p(this);

    float newMin = _minValue;
    float newMax = _maxValue;

    auto mappedCursorPos = mapFromGlobal(QCursor::pos());
    auto highlightedIndex = rect().contains(mappedCursorPos) ? indexAtPoint(mappedCursorPos) : -1;

    auto fontHeight = QFontMetrics(p.font()).height();

    // X-axis
    {
        p.setPen(Qt::gray);
        QPointF pt = dataPoint(0, 0);
        p.drawLine(pt, QPointF(0, pt.y()));
    }

    for (int x = 0; x < _functions.size(); x++) {
        auto* function = _functions[x];

        bool haveLast = false;
        QPointF last;
        if (x == 0) {
            p.setPen(_color);
        } else {
            p.setPen(Qt::red);
        }

        int start = 0;
        if (chartSize < _history->size()) {
            start = static_cast<int>(_history->size()) - chartSize;
        }

        for (int i = 0; i < chartSize; ++i) {
            float v = 0;
            int hist_idx = start + i;
            if (hist_idx < _history->size() && _history->at(hist_idx) &&
                function->value(*_history->at(hist_idx).get(), &v)) {
                if (autoRange) {
                    newMin = min(newMin, v);
                    newMax = max(newMax, v);
                }

                QPointF pt = dataPoint(i, v);

                if (i == highlightedIndex) {
                    p.drawEllipse(pt, 5, 5);

                    p.drawText(mappedCursorPos + QPointF(15, 0 + fontHeight * 2 * x),
                               (" V: " + std::to_string(v)).c_str());

                    if (hist_idx > 0 && hist_idx < _history->size() - 1) {
                        float v1 = 0;
                        float v2 = 0;
                        function->value(*_history->at(hist_idx - 1).get(), &v1);
                        function->value(*_history->at(hist_idx + 1).get(), &v2);

                        auto t1 = _history->at(hist_idx - 1)->timestamp();
                        auto t2 = _history->at(hist_idx + 1)->timestamp();
                        auto deltaTime = RJ::timestamp_to_secs(t2 - t1);

                        auto derivative = (v2 - v1) / (deltaTime);
                        p.drawText(mappedCursorPos + QPointF(15, fontHeight * (1 + x * 2)),
                                   ("dV: " + std::to_string(derivative)).c_str());
                    }
                }

                if (haveLast) {
                    p.drawLine(last, pt);
                }
                last = pt;
                haveLast = true;
            } else {
                haveLast = false;
            }
        }
    }

    p.drawText(0, height() - 5, std::to_string(newMin).c_str());
    p.drawText(0, fontHeight, std::to_string(newMax).c_str());

    _minValue = newMin;
    _maxValue = newMax;
}

////////

bool Chart::PointMagnitude::value(const Packet::LogFrame& frame, float* v) const {
    const Message* msg = &frame;
    for (int i = 0; i < path.size(); ++i) {
        const Reflection* ref = msg->GetReflection();
        const Descriptor* desc = msg->GetDescriptor();

        int tag = path[i];
        const FieldDescriptor* fd = desc->FindFieldByNumber(tag);
        if (fd->type() != FieldDescriptor::TYPE_MESSAGE) {
            std::cerr << "PointMagnitude: expected a message field\n";
            return false;
        }

        if (fd->is_repeated()) {
            ++i;
            if (i >= path.size()) {
                std::cerr << "PointMagnitude: ends after tag for repeated "
                             "field without giving index\n";
                return false;
            }
            int j = path[i];
            if (ref->FieldSize(*msg, fd) <= j) {
                // Not enough items
                return false;
            }
            msg = &ref->GetRepeatedMessage(*msg, fd, j);
        } else {
            if (!ref->HasField(*msg, fd)) {
                // Missing field
                return false;
            }
            msg = &ref->GetMessage(*msg, fd);
        }
    }

    if (msg->GetDescriptor()->name() != "Point") {
        std::cerr << "PointMagnitude: path ended in a message other than Point\n";
        return false;
    }

    *v = static_cast<float>(Geometry2d::Point(*dynamic_cast<const Packet::Point*>(msg)).mag());
    return true;
}

bool Chart::NumericField::value(const Packet::LogFrame& frame, float* v) const {
    const Message* msg = &frame;
    for (int i = 0; i < path.size(); ++i) {
        const Reflection* ref = msg->GetReflection();
        const Descriptor* desc = msg->GetDescriptor();

        int tag = path[i];
        const FieldDescriptor* fd = desc->FindFieldByNumber(tag);
        if (fd->type() != FieldDescriptor::TYPE_MESSAGE) {
            if ((i == path.size() - 1 && !fd->is_repeated()) ||
                (i == path.size() - 2 && fd->is_repeated())) {
                // End of the path
                if (fd->is_repeated()) {
                    ++i;
                    if (i >= path.size()) {
                        std::cerr << "NumericField: ends after tag for "
                                     "repeated field without giving index\n";
                        return false;
                    }
                    int j = path[i];
                    if (ref->FieldSize(*msg, fd) <= j) {
                        // Not enough items
                        return false;
                    }

                    switch (fd->type()) {
                        case FieldDescriptor::TYPE_FLOAT:
                            *v = ref->GetRepeatedFloat(*msg, fd, j);
                            break;

                        case FieldDescriptor::TYPE_DOUBLE:
                            *v = static_cast<float>(ref->GetRepeatedDouble(*msg, fd, j));
                            break;

                        default:
                            std::cerr << "NumericField: unsupported repeated "
                                         "field type "
                                      << fd->type() << "\n";
                            return false;
                    }
                } else {
                    switch (fd->type()) {
                        case FieldDescriptor::TYPE_FLOAT:
                            *v = ref->GetFloat(*msg, fd);
                            break;

                        case FieldDescriptor::TYPE_DOUBLE:
                            *v = static_cast<float>(ref->GetDouble(*msg, fd));
                            break;

                        default:
                            std::cerr << "NumericField: unsupported field type " << fd->type()
                                      << "\n";
                            return false;
                    }
                }
                return true;
            }
            // Non-message field in the middle of a path
            std::cerr << "NumericField: expected a message field\n";
            return false;
        }

        if (fd->is_repeated()) {
            ++i;
            if (i >= path.size()) {
                std::cerr << "NumericField: ends after tag for repeated field "
                             "without giving index\n";
                return false;
            }
            int j = path[i];
            if (ref->FieldSize(*msg, fd) <= j) {
                // Not enough items
                return false;
            }
            msg = &ref->GetRepeatedMessage(*msg, fd, j);
        } else {
            if (!ref->HasField(*msg, fd)) {
                // Missing field
                return false;
            }
            msg = &ref->GetMessage(*msg, fd);
        }
    }

    // Not reached
    return false;
}
