#pragma once

#include <QWidget>
#include <string.h>

#include <vector>
#include <memory>

namespace Packet {
class LogFrame;
}

// Chart functions:
//
// Gets the value for a given frame.
// Stores the value in v.
// Returns true if the value was available.
// Returns false with v undefined if the value was not available.
namespace Chart {
struct Function {
    virtual ~Function() {}
    virtual bool value(const Packet::LogFrame& frame, float& v) const = 0;

    // Vector of tags from LogFrame to the float, double, or point field to be
    // used.
    // Each tag except the last one must identify a Message.
    // A repeated field's tag is followed by the index of the item.
    QVector<int> path;
    QString name;
};

struct PointMagnitude : public Function {
    virtual bool value(const Packet::LogFrame& frame, float& v) const override;
};

struct NumericField : public Function {
    virtual bool value(const Packet::LogFrame& frame, float& v) const override;
};
}

class StripChart : public QWidget {
public:
    StripChart(QWidget* parent = nullptr);
    ~StripChart();

    void history(const std::vector<std::shared_ptr<Packet::LogFrame> >* value) {
        _history = value;
    }

    // Sets the chart function.
    // This chart owns the function and will destroy it when needed.
    void function(Chart::Function* function);

    QList<Chart::Function*> getFunctions() { return _functions; }

    // Exports the contents of the chart to a .csv file
    void exportChart();

    void minValue(float v) { _minValue = v; }

    void maxValue(float v) { _maxValue = v; }

    void color(const QColor& color) {
        _color = color;
        update();
    }

    // If true, minValue and maxValue are automatically changed when
    // out-of-range values are found
    bool autoRange;

    // Arbitrary value for width of the chart, must be <= history size
    int chartSize = 1000;

protected:
    void paintEvent(QPaintEvent* e) override;

    // Returns the position for the data at a given frame.
    // i is an index in _history, so 0 is the most recent frame
    // and increasing indices are older frames.
    QPointF dataPoint(int i, float value);

    int indexAtPoint(const QPoint& point);

    // Chart function (see above)
    QList<Chart::Function*> _functions;

    float _minValue;
    float _maxValue;
    QColor _color;

    const std::vector<std::shared_ptr<Packet::LogFrame> >* _history;

    QPointF _mouse_overlay;
};
