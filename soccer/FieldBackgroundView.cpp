#include "FieldBackgroundView.hpp"

#include <math.h>

FieldBackgroundView::FieldBackgroundView(QWidget* parent)
    : QWidget(parent),
      _fieldOrientation(FieldOrientationPortrait),
      _transformsValid(false), _blueTeam(false), _defendingPlusX(true) {
    // Green background color
    QPalette p = palette();
    p.setColor(QPalette::Window, QColor(0, 85, 0));
    setPalette(p);
    setAutoFillBackground(true);
}

void FieldBackgroundView::setupWorldSpace(QPainter* painter) const {
    painter->translate(width() / 2.0, height() / 2.0);
    painter->scale(width(), -height());
    painter->rotate(static_cast<int>(fieldOrientation()) * 90);
    painter->scale(1.0 / fieldDimensions().FloorLength(), 1.0 / fieldDimensions().FloorWidth());
}

void FieldBackgroundView::setFieldOrientation(FieldOrientation orientation) {
    _fieldOrientation = orientation;

    invalidateTransforms();
    updateGeometry();   //  Fix size
    update();           //  trigger a redraw
}

void FieldBackgroundView::recalculateTransformsIfNeeded() const {
    if (_transformsValid) return;

    _screenToWorld = Geometry2d::TransformMatrix();
    _screenToWorld *= Geometry2d::TransformMatrix::scale(
        fieldDimensions().FloorLength(), fieldDimensions().FloorWidth());
    _screenToWorld *= Geometry2d::TransformMatrix::rotate(
        -static_cast<int>(fieldOrientation()) * M_PI / 2.0);
    _screenToWorld *=
        Geometry2d::TransformMatrix::scale(1.0 / width(), -1.0 / height());
    _screenToWorld *=
        Geometry2d::TransformMatrix::translate(-width() / 2.0, -height() / 2.0);

    _worldToTeam = Geometry2d::TransformMatrix();
    _worldToTeam *= Geometry2d::TransformMatrix::translate(
        0, fieldDimensions().Length() / 2.0f);
    if (_defendingPlusX) {
        _worldToTeam *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
    } else {
        _worldToTeam *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
    }

    _teamToWorld = Geometry2d::TransformMatrix();
    if (_defendingPlusX) {
        _teamToWorld *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
    } else {
        _teamToWorld *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
    }
    _teamToWorld *= Geometry2d::TransformMatrix::translate(
        0, -fieldDimensions().Length() / 2.0f);


    _textRotationWorldSpace = -static_cast<int>(fieldOrientation()) * 90;
    _textRotationTeamSpace =
        _textRotationWorldSpace + (_defendingPlusX ? -90 : 90);

    _transformsValid = true;
}

void FieldBackgroundView::resizeEvent(QResizeEvent* e) {
    int givenW = e->size().width();
    int givenH = e->size().height();
    int needW, needH;
    if (FieldOrientationIsPortrait(fieldOrientation())) {
        needH = roundf(givenW * fieldDimensions().FloorLength() /
                       fieldDimensions().FloorWidth());
        needW = roundf(givenH * fieldDimensions().FloorWidth() /
                       fieldDimensions().FloorLength());
    } else {
        needH = roundf(givenW * fieldDimensions().FloorWidth() /
                       fieldDimensions().FloorLength());
        needW = roundf(givenH * fieldDimensions().FloorLength() /
                       fieldDimensions().FloorWidth());
    }

    QSize size = needW < givenW ? QSize(needW, givenH) : QSize(givenW, needH);

    if (size != e->size()) {
        resize(size);
        invalidateTransforms();
    }
    e->accept();
}

// Draws the field including white lines, goal zones, goals, etc
void FieldBackgroundView::paintEvent(QPaintEvent* e) {
    recalculateTransformsIfNeeded();

    //  setup painter with antialiasing for drastically improved rendering
    //  quality
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    setupWorldSpace(&p);

    //reset to center
    p.translate(-fieldDimensions().FloorLength() /2.0, -fieldDimensions().FloorWidth() /2.0);

    p.translate(fieldDimensions().Border(), fieldDimensions().Border());

    p.setPen(QPen(Qt::white, fieldDimensions().LineWidth() *2.0));  //  double-width pen for visibility (although its less accurate)
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(0, 0, fieldDimensions().Length(), fieldDimensions().Width()));

    //set brush alpha to 0
    p.setBrush(QColor(0,130,0, 0));

    //reset to center
    p.translate(fieldDimensions().Length() /2.0, fieldDimensions().Width() /2.0);

    //centerline
    p.drawLine(QLineF(0, fieldDimensions().Width() /2,0, -fieldDimensions().Width() /2.0));

    //center circle
    p.drawEllipse(QRectF(-fieldDimensions().CenterRadius(), -fieldDimensions().CenterRadius(),
        fieldDimensions().CenterDiameter(), fieldDimensions().CenterDiameter()));

    p.translate(-fieldDimensions().Length() /2.0, 0);

    //goal areas
    p.drawArc(QRectF(-fieldDimensions().ArcRadius(), -fieldDimensions().ArcRadius() + fieldDimensions().GoalFlat() /2.f, 2.f * fieldDimensions().ArcRadius(), 2.f * fieldDimensions().ArcRadius()), -90*16, 90*16);
    p.drawArc(QRectF(-fieldDimensions().ArcRadius(), -fieldDimensions().ArcRadius() - fieldDimensions().GoalFlat() /2.f, 2.f * fieldDimensions().ArcRadius(), 2.f * fieldDimensions().ArcRadius()), 90*16, -90*16);
    p.drawLine(QLineF(fieldDimensions().ArcRadius(), -fieldDimensions().GoalFlat() /2.f, fieldDimensions().ArcRadius(), fieldDimensions().GoalFlat() /2.f));
    // Penalty Mark
    p.drawEllipse(QRectF(-fieldDimensions().PenaltyDiam() /2.0f + fieldDimensions().PenaltyDist(), -fieldDimensions().PenaltyDiam() /2.0f, fieldDimensions().PenaltyDiam(), fieldDimensions().PenaltyDiam()));

    p.translate(fieldDimensions().Length(), 0);

    p.drawArc(QRectF(-fieldDimensions().ArcRadius(), -fieldDimensions().ArcRadius() + fieldDimensions().GoalFlat() /2.f, 2.f * fieldDimensions().ArcRadius(), 2.f * fieldDimensions().ArcRadius()), -90*16, -90*16);
    p.drawArc(QRectF(-fieldDimensions().ArcRadius(), -fieldDimensions().ArcRadius() - fieldDimensions().GoalFlat() /2.f, 2.f * fieldDimensions().ArcRadius(), 2.f * fieldDimensions().ArcRadius()), 90*16, 90*16);
    p.drawLine(QLineF(-fieldDimensions().ArcRadius(), -fieldDimensions().GoalFlat() /2.f, -fieldDimensions().ArcRadius(), fieldDimensions().GoalFlat() /2.f));
    // Penalty Mark
    p.drawEllipse(QRectF(-fieldDimensions().PenaltyDiam() /2.0f - fieldDimensions().PenaltyDist(), -fieldDimensions().PenaltyDiam() /2.0f, fieldDimensions().PenaltyDiam(), fieldDimensions().PenaltyDiam()));

    // goals
    float x[2] = {0, fieldDimensions().GoalDepth()};
    float y[2] = {fieldDimensions().GoalWidth() /2.0f, -fieldDimensions().GoalWidth() /2.0f};

    bool flip = _blueTeam ^ _defendingPlusX;

    QColor goalColor = flip ? Qt::yellow : Qt::blue;
    p.setPen(QPen(goalColor, fieldDimensions().LineWidth() * 2.0)); //  double-width for visibility, not real-life accuracy
    p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
    p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
    p.drawLine(QLineF(x[1], y[1], x[1], y[0]));

    x[0] -= fieldDimensions().Length();
    x[1] -= fieldDimensions().Length() + 2 * fieldDimensions().GoalDepth();

    goalColor = flip ? Qt::blue : Qt::yellow;
    p.setPen(QPen(goalColor, fieldDimensions().LineWidth() * 2.0));
    p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
    p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
    p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
}
