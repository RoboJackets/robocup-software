#include "RobotWidget.hpp"
#include <VisionDotPattern.hpp>
#include <Constants.hpp>
#include <Geometry2d/util.h>
#include <stdexcept>


RobotWidget::RobotWidget(QWidget *parent, Qt::WindowFlags f) {
    for (int i = 0; i < 4; i++) setWheelFault(i, false);
    setBallSenseFault(false);

    _blueTeam = true;
    _shellID = 0;

    _hasBall = false;
    _ballSenseFault = false;
    for (int i = 0; i < 4; i++) _wheelFaults[i] = false;
}

void RobotWidget::setShellID(int shellID) {
    _shellID = shellID;
}

void RobotWidget::setBlueTeam(bool blueTeam) {
    if (blueTeam != _blueTeam) {
        _blueTeam = blueTeam;
        update();
    }
}

bool RobotWidget::blueTeam() const {
    return _blueTeam;
}


void RobotWidget::setWheelFault(int wheelIndex, bool faulty) {
    if (wheelIndex < 0 || wheelIndex > 3) {
        throw std::out_of_range("Invalid wheel index");
    }

    _wheelFaults[wheelIndex] = faulty;
    update();
}

void RobotWidget::setBallSenseFault(bool faulty) {
    _ballSenseFault = faulty;
}

void RobotWidget::setHasBall(bool hasBall) {
    if (hasBall != _hasBall) {
        _hasBall = hasBall;
        update();
    }
}

//  draws a red X with @width = @height = @size centered at @center
void drawRedX(QPainter &painter, const QPointF &center, float size, float lineThickness = 0.01) {
    float halfLen = 0.5*sqrtf(powf(size, 2) + powf(size, 2));

    painter.save(); {
        QPen xPen(Qt::red, lineThickness);
        xPen.setCapStyle(Qt::RoundCap);
        painter.setPen(xPen);

        painter.translate(center);

        painter.rotate(45);
        painter.drawLine(QPointF(-halfLen/2, 0), QPointF(halfLen/2, 0));

        painter.rotate(90);
        painter.drawLine(QPointF(-halfLen/2, 0), QPointF(halfLen/2, 0));
    } painter.restore();
}

void RobotWidget::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    //  move to middle
    painter.translate(rect().center());

    //  scale so we can draw robot in units of meters
    float minPadding = 9;
    float scale = fmin((width() - minPadding*2) / Robot_Radius, (height() - minPadding*2) / Robot_Radius) / 2;
    painter.scale(scale, scale);


    //  draw robot body
    //  note: angles are given to drawChord in sixteenths of a degree
    int span = 40;
    int start = (90+span)*16;
    int end = (360 - span*2)*16;
    painter.setBrush(Qt::black);
    painter.setPen(Qt::NoPen);
    painter.drawChord(QRectF(-Robot_Radius, -Robot_Radius, Robot_Radius * 2, Robot_Radius * 2), start, end);


    //  draw dots
    painter.setPen(Qt::NoPen);
    for (int i = 0; i < 4; i++) {
        painter.setBrush(QBrush(Dot_Pattern_Colors[_shellID][i]));
        QPointF dotCenter;
        dotCenter.setX( (i >= 2) ? Dots_Small_Offset : Dots_Large_Offset );
        dotCenter.setX( dotCenter.x() * ( (i == 1 || i == 2) ? -1 : 1 ) );
        dotCenter.setY( (i <= 1) ? Dots_Small_Offset : Dots_Large_Offset );
        dotCenter.setY( dotCenter.y() * ( (i <= 1) ? -1 : 1 ) );

        painter.drawEllipse(dotCenter, Dots_Radius, Dots_Radius);
    }

    //  draw center dot
    painter.setBrush(_blueTeam ? Qt::blue : Qt::yellow);
    painter.drawEllipse(QPointF(0, 0), Dots_Radius, Dots_Radius);


    const float RedXSize = 0.06;


    //  draw wheels
    const float wheelWidth = 0.015;
    const float wheelRadius = 0.03;
    const float wheelDist = Robot_Radius + wheelWidth / 2;
    const float wheelAngles[] = {
        -M_PI*0.2,
        -M_PI*(1-0.2),
        -M_PI*(1+0.2),
        -M_PI*(-0.2)
    };
    for (int i = 0; i < 4; i++) {
        painter.save(); {
            float angle = wheelAngles[i];

            //  translate to center of wheel
            painter.translate(wheelDist*cosf(angle), wheelDist*sinf(angle));

            //  FIXME: draw wheel fault
            if (_wheelFaults[i]) {
                painter.save(); {
                    bool right = i == 0 || i == 3;
                    bool front = i == 0 || i == 1;
                    float dist = RedXSize/2;
                    painter.translate(dist * (right ? 1 : -1), dist * (front ? -1 : 1));
                    drawRedX(painter, QPointF(0, 0), RedXSize);
                } painter.restore();
            }


            //  rotate to alight the x-axis with the wheel radius
            painter.rotate((angle+M_PI/2) * RadiansToDegrees);

            painter.setBrush(Qt::gray);
            const float wheelRounding = 0.01;
            painter.drawRoundedRect(QRectF(-wheelRadius, -wheelWidth / 2, wheelRadius * 2, wheelWidth), wheelRounding, wheelRounding);
        } painter.restore();
    }



    if (_ballSenseFault) {
        //  draw a red X by the robot's mouth

        drawRedX(painter, QPointF(0, -Robot_Radius-(RedXSize/2)+0.02), RedXSize);
    } else if (_hasBall) {
        //  draw orange golf ball
        
        const float ballRadius = 0.02135;
        static QColor ballColor(0xff, 0x90, 0);
        float ballCenterY = -(Robot_Radius+ballRadius)+0.02;

        painter.save(); {
            painter.translate(0, ballCenterY);
            painter.setBrush(ballColor);
            painter.drawEllipse(QRectF(-ballRadius, -ballRadius, ballRadius*2, ballRadius*2));
        } painter.restore();
    }
}
