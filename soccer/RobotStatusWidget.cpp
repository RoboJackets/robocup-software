#include "RobotStatusWidget.hpp"


RobotStatusWidget::RobotStatusWidget(QWidget *parent, Qt::WindowFlags f) : QWidget(parent, f) {
    _ui.setupUi(this);


    _shellID = -2;
    setShellID(-1);

    _hasRadio = true;
    setHasRadio(false);

    _hasVision = true;
    setHasVision(false);

    _batteryLevel = 1;
    setBatteryLevel(0);
}

int RobotStatusWidget::shellID() const {
    return _shellID;
}

void RobotStatusWidget::setShellID(int shellID) {
    if (shellID != _shellID) {
        _shellID = shellID;

        if (shellID == -1) {
            _ui.shellID->setText(QString("?"));
        } else {
            _ui.shellID->setText(QString("%1").arg(shellID));
        }
    }
}

bool RobotStatusWidget::hasRadio() const {
    return _hasRadio;
}

void RobotStatusWidget::setHasRadio(bool hasRadio) {
    if (hasRadio != _hasRadio) {
        _hasRadio = hasRadio;

        _ui.radioIndicator->load(QString(hasRadio ? ":radio-connected" : ":radio-disconnected"));
    }
}

bool RobotStatusWidget::hasVision() const {
    return _hasVision;
}

void RobotStatusWidget::setHasVision(bool hasVision) {
    if (hasVision != _hasVision) {
        _hasVision = hasVision;

        _ui.visionIndicator->load(QString(hasVision ? ":vision-available" : ":vision-unavailable"));
    }
}

float RobotStatusWidget::batteryLevel() const {
    return _batteryLevel;
}

void RobotStatusWidget::setBatteryLevel(float batteryLevel) {
    if (abs(batteryLevel - _batteryLevel) > 0.01) {
        _batteryLevel = batteryLevel;
        _ui.batteryIndicator->setValue(batteryLevel);
    }
}
