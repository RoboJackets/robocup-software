#include "RobotStatusWidget.hpp"


RobotStatusWidget::RobotStatusWidget(QWidget *parent, Qt::WindowFlags f) : QWidget(parent, f) {
    _ui.setupUi(this);

    setBoardID("?\?-??");

    _shellID = -2;
    setShellID(-1);

    _blueTeam = false;
    setBlueTeam(true);

    _hasRadio = true;
    setHasRadio(false);

    _hasVision = true;
    setHasVision(false);

    _batteryLevel = 1;
    setBatteryLevel(0.5);

    _showstopper = false;
}

int RobotStatusWidget::shellID() const {
    return _shellID;
}

void RobotStatusWidget::setShellID(int shellID) {
    if (shellID != _shellID) {
        _shellID = shellID;

        _ui.robotWidget->setShellID(_shellID);

        if (shellID == -1) {
            _ui.shellID->setText(QString("?"));
        } else {
            _ui.shellID->setText(QString("%1").arg(shellID));
        }
    }
}

void RobotStatusWidget::setBlueTeam(bool blueTeam) {
    _ui.robotWidget->setBlueTeam(blueTeam);
    _blueTeam = blueTeam;
}

bool RobotStatusWidget::blueTeam() const {
    return _blueTeam;
}

const QString &RobotStatusWidget::boardID() const {
    return _boardID;
}

void RobotStatusWidget::setBoardID(const QString &boardID) {
    if (boardID != _boardID) {
        _boardID = boardID;
        _ui.boardID->setText(QString("Board ID: %1").arg(boardID));
    }
}

void RobotStatusWidget::setWheelFault(int wheelIndex, bool faulty) {
    _ui.robotWidget->setWheelFault(wheelIndex, faulty);
}

void RobotStatusWidget::setBallSenseFault(bool faulty) {
    _ui.robotWidget->setBallSenseFault(faulty);
}

void RobotStatusWidget::setHasBall(bool hasBall) {
    _ui.robotWidget->setHasBall(hasBall);
}


bool RobotStatusWidget::hasRadio() const {
    return _hasRadio;
}

void RobotStatusWidget::setHasRadio(bool hasRadio) {
    if (hasRadio != _hasRadio) {
        _hasRadio = hasRadio;

        _ui.radioIndicator->setPixmap(QPixmap(QString(hasRadio ? ":icons/radio-connected.svg" : ":icons/radio-disconnected.svg")));
    }
}

bool RobotStatusWidget::hasVision() const {
    return _hasVision;
}

void RobotStatusWidget::setHasVision(bool hasVision) {
    if (hasVision != _hasVision) {
        _hasVision = hasVision;

        _ui.visionIndicator->setPixmap(QPixmap(QString(hasVision ? ":icons/vision-available.svg" : ":icons/vision-unavailable.svg")));
    }
}
 
float RobotStatusWidget::batteryLevel() const {
    return _batteryLevel;
}

void RobotStatusWidget::setBatteryLevel(float batteryLevel) {
    if (fabs(batteryLevel - _batteryLevel) > 0.01) {
        _batteryLevel = batteryLevel;
        _ui.batteryIndicator->setBatteryLevel(_batteryLevel);
    }
}

void RobotStatusWidget::setShowstopper(bool showstopper) {
    if (showstopper != _showstopper) {
        _showstopper = showstopper;
        _ui.shellID->setStyleSheet(_showstopper ? "color: red;" : "");
    }
}
