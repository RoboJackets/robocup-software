#include "rj_ui/field_view.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <set>

#include <QGLWidget>
#include <QLabel>
#include <QLayout>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QStyle>
#include <QStyleOption>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/motion_constraints.hpp>
#include <rj_common/vision_dot_pattern.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/transform_matrix.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/log_utils.hpp>

using namespace std;

using namespace boost;

static QPen redPen(Qt::red, 0);
static QPen bluePen(Qt::blue, 0);
static QPen yellowPen(Qt::yellow, 0);
static QPen blackPen(Qt::black, 0);
static QPen whitePen(Qt::white, 0);
static QPen greenPen(Qt::green, 0);
static QPen grayPen(Qt::gray, 0);
static QPen darkRedPen(Qt::darkRed, 0);

static QPen tempPen(Qt::white, 0);

static QColor ballColor(0xff, 0x90, 0);
static QPen ballPen(ballColor, 0);

FieldView::FieldView(QWidget* parent) : QWidget(parent) {
    showRawRobots = false;
    showRawBalls = false;
    showCoords = false;
    showDotPatterns = false;
    showTeamNames = false;
    _rotate = 1;

    // Green background
    QPalette p = palette();
    p.setColor(QPalette::Window, QColor(0, 85.0, 0));
    setPalette(p);
    setAutoFillBackground(true);

    // Initialize the label and cursor for hovering display
    _posLabel = new QLabel(this);
    QRect rect = QFontMetrics(_posLabel->font()).boundingRect("X: -9.99, Y: -9.99");
    _posLabel->setMinimumWidth(rect.width());
    _posLabel->setStyleSheet("QLabel { color: red; background: none;}");

    // enable mouse tracking so we can update position label
    setMouseTracking(true);

    show();
}

void FieldView::leaveEvent(QEvent* /*event*/) { _posLabel->setVisible(false); }

void FieldView::enterEvent(QEvent* /*event*/) { _posLabel->setVisible(true); }

void FieldView::mouseMoveEvent(QMouseEvent* me) {
    _posLabel->move(QPoint(me->pos().x() - 45, me->pos().y() + 17));
    rj_geometry::Point pos = _worldToTeam * _screenToWorld * me->pos();
    QString s = "X: ";
    s += QString::number(std::round(pos.x() * 100) / 100);
    s += " Y: ";
    s += QString::number(std::round(pos.y() * 100) / 100);
    _posLabel->setText(s);
}

void FieldView::rotate(int value) {
    _rotate = value;

    // Fix size
    updateGeometry();

    update();
}

void FieldView::paintEvent(QPaintEvent* /*e*/) {
    QPainter p(this);
    QStyleOption opt;
    opt.init(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);

    // antialiasing drastically improves rendering quality
    p.setRenderHint(QPainter::Antialiasing);

    if (!live) {
        // Non-live border
        p.setPen(QPen(Qt::red, 4));
        p.drawRect(rect());
    }

    // Set up world space
    p.translate(width() / 2.0, height() / 2.0);
    p.scale(width(), -height());
    p.rotate(_rotate * 90);
    p.scale(1.0 / FieldDimensions::current_dimensions.floor_length(),
            1.0 / FieldDimensions::current_dimensions.floor_width());

    // Set text rotation for world space
    _textRotation = -_rotate * 90;

    if (showCoords) {
        drawCoords(p);
    }
}

void FieldView::drawWorldSpace(QPainter& p) {
}

void FieldView::drawTeamSpace(QPainter& p) {
}

void FieldView::drawText(QPainter& p, QPointF pos, const QString& text, bool center) const {
    p.save();
    p.translate(pos);
    p.rotate(_textRotation);
    p.scale(0.0131, -0.0131);

    if (center) {
        int flags = Qt::AlignHCenter | Qt::AlignVCenter;
        QRectF r = p.boundingRect(QRectF(), flags, text);
        p.drawText(r, flags, text);
    } else {
        p.drawText(QPointF(), text);
    }

    p.restore();
}

void FieldView::drawCoords(QPainter& p) {
    p.setPen(grayPen);

    // X
    p.drawLine(QPointF(0, 0), QPointF(0.25, 0));
    p.drawLine(QPointF(0.25, 0), QPointF(0.20, -0.05));
    p.drawLine(QPointF(0.25, 0), QPointF(0.20, 0.05));
    drawText(p, QPointF(0.25, 0.1), "+X");

    // Y
    p.drawLine(QPointF(0, 0), QPointF(0, 0.25));
    p.drawLine(QPointF(0, 0.25), QPointF(-0.05, 0.20));
    p.drawLine(QPointF(0, 0.25), QPointF(0.05, 0.20));
    drawText(p, QPointF(0.1, 0.25), "+Y");
}

void FieldView::drawRobot(QPainter& painter, bool blueRobot, int ID, QPointF pos, float theta,
                          bool hasBall, bool faulty) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::NoBrush);

    painter.save();

    painter.translate(pos.x(), pos.y());

    if (faulty) {
        painter.save();
        painter.setPen(redPen);
        painter.setBrush(QBrush{Qt::red, Qt::SolidPattern});
        auto r = kRobotRadius + 0.025;
        painter.drawEllipse(QPointF(0, 0), r, r);
        painter.restore();
    }

    if (blueRobot) {
        painter.setPen(bluePen);
        painter.setBrush(Qt::blue);
    } else {
        painter.setPen(yellowPen);
        painter.setBrush(Qt::yellow);
    }

    painter.rotate(radians_to_degrees(theta) + 90);

    int span = 40;

    int start = span * 16 + 90 * 16;
    int end = 360 * 16 - (span * 2) * 16;
    const float r = kRobotRadius;
    painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);

    if (showDotPatterns) {
        painter.setPen(Qt::NoPen);
        for (int i = 0; i < 4; i++) {
            painter.setBrush(QBrush(kDotPatternColors[ID][i]));
            QPointF center;
            center.setX((i >= 2) ? kDotsSmallOffset : kDotsLargeOffset);
            center.setX(center.x() * ((i == 1 || i == 2) ? -1 : 1));
            center.setY((i <= 1) ? kDotsSmallOffset : kDotsLargeOffset);
            center.setY(center.y() * ((i <= 1) ? -1 : 1));
            painter.drawEllipse(center, kDotsRadius, kDotsRadius);
        }
    }

    if (hasBall) {
        painter.setPen(redPen);
        const float r = kRobotRadius * 0.75f;
        painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
    }

    painter.restore();

    // draw shell number
    painter.save();
    painter.translate(pos.x(), pos.y());
    if (blueRobot) {
        painter.setPen(whitePen);
    } else {
        painter.setPen(blackPen);
    }
    drawText(painter, QPointF(), QString::number(ID));
    painter.restore();

    drawRobotHeading(&painter, pos, theta);
}

void FieldView::drawRobotHeading(QPainter* painter, QPointF pos, float theta,
                                 const QColor& heading_color, float heading_line_len) {
    painter->save();
    painter->translate(pos.x(), pos.y());
    painter->rotate(radians_to_degrees(theta));

    const QPen heading_pen(heading_color, 0);
    painter->setPen(heading_pen);

    const QPointF start_pt{0, 0};
    const QPointF end_pt{heading_line_len, 0};
    const QLineF heading_line{start_pt, end_pt};

    painter->drawLine(heading_line);
    painter->restore();
}

void FieldView::resizeEvent(QResizeEvent* e) {
    int givenW = e->size().width();
    int givenH = e->size().height();
    int needW;
    int needH;
    if ((_rotate & 1) != 0) {
        needH = static_cast<int>(std::round(static_cast<float>(givenW) *
                                            FieldDimensions::current_dimensions.floor_length() /
                                            FieldDimensions::current_dimensions.floor_width()));
        needW = static_cast<int>(std::round(static_cast<float>(givenH) *
                                            FieldDimensions::current_dimensions.floor_width() /
                                            FieldDimensions::current_dimensions.floor_length()));
    } else {
        needH = static_cast<int>(std::round(static_cast<float>(givenW) *
                                            FieldDimensions::current_dimensions.floor_width() /
                                            FieldDimensions::current_dimensions.floor_length()));
        needW = static_cast<int>(std::round(static_cast<float>(givenH) *
                                            FieldDimensions::current_dimensions.floor_length() /
                                            FieldDimensions::current_dimensions.floor_width()));
    }

    QSize size;
    if (needW < givenW) {
        size = QSize(needW, givenH);
    } else {
        size = QSize(givenW, needH);
    }

    if (size != e->size()) {
        resize(size);
    }
    e->accept();
}
