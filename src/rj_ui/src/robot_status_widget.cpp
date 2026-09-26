#include "rj_ui/robot_status_widget.hpp"

#include "ui_RobotStatusWidget.h"

RobotStatusWidget::RobotStatusWidget(QWidget* parent, Qt::WindowFlags f) : QWidget(parent, f) {
    _ui.setupUi(this);
    setBoardID("RJ");
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
    _ui.kickerIndicator->setText(QString("K"));
    _kickerState = false;
    setKickerState(false);
}

int RobotStatusWidget::shellID() const { return _shellID; }

void RobotStatusWidget::setShellID(int shellID) {
    if (shellID != _shellID) {
        _shellID = shellID;
        _ui.robotWidget->setShellID(_shellID);
        _ui.shellID->setText(shellID == -1 ? QString("?") : QString("%1").arg(shellID));
    }
}

void RobotStatusWidget::setErrorText(const QString& error) { _ui.errorText->setText(error); }

void RobotStatusWidget::setBlueTeam(bool blueTeam) {
    _ui.robotWidget->setBlueTeam(blueTeam);
    _blueTeam = blueTeam;
}

bool RobotStatusWidget::blueTeam() const { return _blueTeam; }
const QString& RobotStatusWidget::boardID() const { return _boardID; }

void RobotStatusWidget::setBoardID(const QString& boardID) {
    if (boardID != _boardID) {
        _boardID = boardID;
        _ui.boardID->setText(QString("ID: %1").arg(boardID));
    }
}

QString RobotStatusWidget::robotModel() const { return _ui.robotModel->text(); }
void RobotStatusWidget::setRobotModel(const QString& robotModel) {
    _ui.robotModel->setText(robotModel);
}
void RobotStatusWidget::setWheelFault(int wheelIndex, bool faulty) {
    _ui.robotWidget->setWheelFault(wheelIndex, faulty);
}
void RobotStatusWidget::setBallSenseFault(bool faulty) {
    _ui.robotWidget->setBallSenseFault(faulty);
}
void RobotStatusWidget::setHasBall(bool hasBall) { _ui.robotWidget->setHasBall(hasBall); }
bool RobotStatusWidget::hasRadio() const { return _hasRadio; }

void RobotStatusWidget::setHasRadio(bool hasRadio) {
    if (hasRadio != _hasRadio) {
        _hasRadio = hasRadio;
        _ui.radioIndicator->setPixmap(QPixmap(
            QString(hasRadio ? ":icons/radio-connected.svg" : ":icons/radio-disconnected.svg")));
    }
}

bool RobotStatusWidget::hasVision() const { return _hasVision; }

void RobotStatusWidget::setHasVision(bool hasVision) {
    if (hasVision != _hasVision) {
        _hasVision = hasVision;
        _ui.visionIndicator->setPixmap(QPixmap(
            QString(hasVision ? ":icons/vision-available.svg" : ":icons/vision-unavailable.svg")));
    }
}

float RobotStatusWidget::batteryLevel() const { return _batteryLevel; }

void RobotStatusWidget::setBatteryLevel(float batteryLevel) {
    if (std::fabs(batteryLevel - _batteryLevel) > 0.01) {
        _batteryLevel = batteryLevel;
        _ui.batteryIndicator->setBatteryLevel(_batteryLevel);
    }
}

bool RobotStatusWidget::kickerState() const { return _kickerState; }

void RobotStatusWidget::setKickerState(bool kickerState) {
    _kickerState = kickerState;
    _ui.kickerIndicator->setStyleSheet(kickerState ? "QLabel { color : green; }"
                                                   : "QLabel { color : red; }");
}

void RobotStatusWidget::setShowstopper(bool showstopper) {
    if (showstopper != _showstopper) {
        _showstopper = showstopper;
        _ui.shellID->setStyleSheet(_showstopper ? "color: red;" : "");
    }
}

void RobotStatusWidget::load(const RobotStatus& status,
                             const std::optional<rj_common::UIRobot>& maybe_robot,
                             bool blueTeam) {
    setShellID(status.shell_id);
    setBlueTeam(blueTeam);
    setRobotModel(status.version == RobotStatus::HardwareVersion::kSimulated   ? "Simulation"
                  : status.version == RobotStatus::HardwareVersion::kFleet2018 ? "RJ2018"
                                                                               : "Unknown Bot");
    setHasRadio(status.timestamp != RJ::Time{});
    setHasVision(maybe_robot.has_value());

    QStringList errors;
    bool motorFault = false;
    for (size_t i = 0; i < status.motors_healthy.size(); ++i) {
        const bool faulty = !status.motors_healthy[i];
        motorFault = motorFault || faulty;
        if (i < 4)
            setWheelFault(static_cast<int>(i), faulty);
        else
            setBallSenseFault(faulty);
        if (faulty) errors << QString("Motor Fault %1").arg(static_cast<int>(i));
    }

    const bool kickerFault = status.kicker == RobotStatus::KickerState::kFailed;
    setKickerState(status.kicker == RobotStatus::KickerState::kCharged);
    if (kickerFault) errors << "Kicker Fault";
    setHasBall(status.has_ball);

    const float battery =
        status.version == RobotStatus::HardwareVersion::kSimulated
            ? 1.0f
            : static_cast<float>(kRJ2015BatteryProfile.get_charge_level(status.battery_voltage));
    setBatteryLevel(battery);
    setErrorText(errors.join(", "));
    setShowstopper(!maybe_robot.has_value() || motorFault || kickerFault || battery < 0.25f ||
                   !status.fpga_healthy);
}
