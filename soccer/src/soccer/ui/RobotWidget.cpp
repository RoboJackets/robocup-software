#include "RobotWidget.hpp"

#include <cmath>
#include <stdexcept>

#include <Geometry2d/Util.hpp>
#include <rj_common/VisionDotPattern.hpp>
#include <rj_constants/constants.hpp>

RobotWidget::RobotWidget(QWidget* /*parent*/, Qt::WindowFlags /*f*/) {
    for (int i = 0; i < 4; i++) {
        setWheelFault(i, false);
    }
    setBallSenseFault(false);

    _blueTeam = true;
    _shellID = 0;

    _hasBall = false;
    _ballSenseFault = false;
    for (bool& wheel_fault : _wheelFaults) {
        wheel_fault = false;
    }
}

void RobotWidget::setShellID(int shell_id) { _shellID = shell_id; }

void RobotWidget::setBlueTeam(bool blue_team) {
    if (blue_team != _blueTeam) {
        _blueTeam = blue_team;
        update();
    }
}

bool RobotWidget::blueTeam() const { return _blueTeam; }

void RobotWidget::setWheelFault(int wheel_index, bool faulty) {
    if (wheel_index < 0 || wheel_index > 3) {
        throw std::out_of_range("Invalid wheel index");
    }

    _wheelFaults[wheel_index] = faulty;
    update();
}

void RobotWidget::setBallSenseFault(bool faulty) { _ballSenseFault = faulty; }

void RobotWidget::setHasBall(bool has_ball) {
    if (has_ball != _hasBall) {
        _hasBall = has_ball;
        update();
    }
}

//  draws a red X with @width = @height = @size centered at @center
void drawRedX(QPainter& painter, const QPointF& center, float size,
              float line_thickness = 0.01) {
    float half_len = 0.5 * sqrtf(powf(size, 2) + powf(size, 2));

    painter.save();
    {
        QPen x_pen(Qt::red, line_thickness);
        x_pen.setCapStyle(Qt::RoundCap);
        painter.setPen(x_pen);

        painter.translate(center);

        painter.rotate(45);
        painter.drawLine(QPointF(-half_len / 2, 0), QPointF(half_len / 2, 0));

        painter.rotate(90);
        painter.drawLine(QPointF(-half_len / 2, 0), QPointF(half_len / 2, 0));
    }
    painter.restore();
}

void RobotWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    //  move to middle
    painter.translate(rect().center());

    //  scale so we can draw robot in units of meters
    float min_padding = 9;
    float scale = std::fmin((width() - min_padding * 2) / Robot_Radius,
                            (height() - min_padding * 2) / Robot_Radius) /
                  2;
    painter.scale(scale, scale);

    //  draw robot body
    //  note: angles are given to drawChord in sixteenths of a degree
    int span = 40;
    int start = (90 + span) * 16;
    int end = (360 - span * 2) * 16;
    painter.setBrush(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.drawChord(QRectF(-Robot_Radius, -Robot_Radius, Robot_Radius * 2,
                             Robot_Radius * 2),
                      start, end);

    //  draw dots
    painter.setPen(Qt::NoPen);
    for (int i = 0; i < 4; i++) {
        painter.setBrush(QBrush(Dot_Pattern_Colors[_shellID][i]));
        QPointF dot_center;
        dot_center.setX((i >= 2) ? Dots_Small_Offset : Dots_Large_Offset);
        dot_center.setX(dot_center.x() * ((i == 1 || i == 2) ? 1 : -1));
        dot_center.setY((i <= 1) ? Dots_Small_Offset : Dots_Large_Offset);
        dot_center.setY(dot_center.y() * ((i <= 1) ? -1 : 1));

        painter.drawEllipse(dot_center, Dots_Radius, Dots_Radius);
    }

    //  draw center dot
    painter.setBrush(_blueTeam ? Qt::blue : Qt::yellow);
    painter.drawEllipse(QPointF(0, 0), Dots_Radius, Dots_Radius);

    const float red_x_size = 0.06;

    //  draw wheels
    const float wheel_width = 0.015;
    const float wheel_radius = 0.03;
    const float wheel_dist = Robot_Radius + wheel_width / 2;
    const float wheel_angles[] = {-M_PI * 0.8, M_PI * 0.8, M_PI * 0.2,
                                  M_PI * -0.2};

    for (int i = 0; i < 4; i++) {
        painter.save();
        {
            float angle = wheel_angles[i];

            //  translate to center of wheel
            painter.translate(wheel_dist * cosf(angle),
                              wheel_dist * sinf(angle));

            //  FIXME: draw wheel fault
            if (_wheelFaults[i]) {
                painter.save();
                {
                    float dist = red_x_size / 2;
                    painter.translate(cosf(angle) * dist, sinf(angle) * dist);
                    drawRedX(painter, QPointF(0, 0), red_x_size);
                }
                painter.restore();
            }

            //  rotate to alight the x-axis with the wheel radius
            painter.rotate(RadiansToDegrees(angle) + 90);

            painter.setBrush(Qt::gray);
            const float wheel_rounding = 0.01;
            painter.drawRoundedRect(QRectF(-wheel_radius, -wheel_width / 2,
                                           wheel_radius * 2, wheel_width),
                                    wheel_rounding, wheel_rounding);
        }
        painter.restore();
    }

    if (_ballSenseFault) {
        //  draw a red X by the robot's mouth

        drawRedX(painter, QPointF(0, -Robot_Radius - (red_x_size / 2) + 0.02),
                 red_x_size);
    } else if (_hasBall) {
        //  draw orange golf ball

        const float ball_radius = 0.02135;
        static QColor ball_color(0xff, 0x90, 0);
        float ball_center_y = -(Robot_Radius + ball_radius) + 0.02;

        painter.save();
        {
            painter.translate(0, ball_center_y);
            painter.setBrush(ball_color);
            painter.drawEllipse(QRectF(-ball_radius, -ball_radius,
                                       ball_radius * 2, ball_radius * 2));
        }
        painter.restore();
    }
}
