#include "robot_status_widget.hpp"

#include <cmath>
#include <iostream>

#include <battery_profile.hpp>
#include <rj_common/status.hpp>

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

        if (shellID == -1) {
            _ui.shellID->setText(QString("?"));
        } else {
            _ui.shellID->setText(QString("%1").arg(shellID));
        }
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

    if (kickerState) {
        _ui.kickerIndicator->setStyleSheet("QLabel { color : green; }");
    } else {
        _ui.kickerIndicator->setStyleSheet("QLabel { color : red; }");
    }
}

void RobotStatusWidget::setShowstopper(bool showstopper) {
    if (showstopper != _showstopper) {
        _showstopper = showstopper;
        _ui.shellID->setStyleSheet(_showstopper ? "color: red;" : "");
    }
}

void RobotStatusWidget::loadFromLogFrame(const Packet::RadioRx& rx,
                                         const std::optional<Packet::LogFrame::Robot>& maybe_robot,
                                         bool blueTeam) {
    // Set shell ID
    setShellID(rx.robot_id());

    // Set team
    setBlueTeam(blueTeam);

    // set robot model
    QString robotModel;
    switch (rx.hardware_version()) {
        case Packet::HardwareVersion::RJ2008:
            robotModel = "RJ2008";
            break;
        case Packet::HardwareVersion::RJ2011:
            robotModel = "RJ2011";
            break;
        case Packet::HardwareVersion::RJ2015:
            robotModel = "RJ2015";
            break;
        case Packet::HardwareVersion::RJ2018:
            robotModel = "RJ2018";
            break;
        case Packet::HardwareVersion::Simulation:
            robotModel = "Simulation";
            break;
        default:
            robotModel = "Unknown Bot";
    }
    setRobotModel(robotModel);

    // Radio is always true if we're connected to it...
    setHasRadio(true);

    // vision status
    bool hasVision = maybe_robot.has_value();
    setHasVision(hasVision);

    // build a list of errors to display in the widget
    QStringList errorList;

    // Each motor fault is shown as text in the error text display
    // as well as being drawn as a red X on the graphic of a robot
    bool hasMotorFault = false;
    if (rx.motor_status().size() == 5) {
        std::array<const char*, 5> motorNames = {"FL", "BL", "BR", "FR", "Dribbler"};

        // Examine status of each motor (including the dribbler)
        for (int i = 0; i < 5; ++i) {
            bool motorFault = rx.motor_status(i) != Packet::MotorStatus::Good;

            if (motorFault) {
                errorList << QString("Motor Fault %1").arg(motorNames[i]);
                motorFault = true;
            }

            if (i != 4) {
                // Show wheel faults (exluding dribbler, which is index 4)
                setWheelFault(i, motorFault);
            } else {
                // Dribbler fault shows up as ball sense fault.
                setBallSenseFault(motorFault);
            }

            hasMotorFault = hasMotorFault || motorFault;
        }
    } else {
        std::cerr << "Expected 5 motors, but only " << rx.motor_status_size() << std::endl;
        hasMotorFault = true;
    }

    // Kicker
    bool kickerFault = true;
    bool kickerCharging = false;
    if (rx.has_kicker_status()) {
        kickerFault = (rx.kicker_status() & Kicker_Enabled) == 0u;
        kickerCharging = (rx.kicker_status() & Kicker_Charged) == 0u;
    }
    if (kickerFault) {
        errorList << "Kicker Fault";
    }
    setKickerState(kickerCharging);

    // Ball sense
    bool ballSenseFault = true;
    if (rx.has_ball_sense_status()) {
        ballSenseFault = !(rx.ball_sense_status() == Packet::NoBall ||
                           rx.ball_sense_status() == Packet::HasBall);
    }
    if (ballSenseFault) {
        errorList << "Ball Sense Fault";
    }
    setBallSenseFault(ballSenseFault);

    // FPGA
    bool fpgaWorking = true;
    if (rx.has_fpga_status() && rx.fpga_status() != Packet::FpgaGood) {
        if (rx.fpga_status() == Packet::FpgaNotInitialized) {
            errorList << "FPGA not initialized";
        } else {
            errorList << "FPGA error";
        }
        fpgaWorking = false;
    }

    // Display error text
    setErrorText(errorList.join(", "));

    // Show the ball in the robot's mouth if it has one
    bool hasBall = rx.has_ball_sense_status() && rx.ball_sense_status() == Packet::HasBall;
    setHasBall(hasBall);

    // Convert battery voltage to a percentage and show it with the battery
    // indicator
    float batteryLevel = 1;
    if (rx.has_battery()) {
        if (rx.hardware_version() == Packet::RJ2008 || rx.hardware_version() == Packet::RJ2011) {
            batteryLevel = static_cast<float>(kRJ2008BatteryProfile.get_charge_level(rx.battery()));
        } else if (rx.hardware_version() == Packet::RJ2015 ||
                   rx.hardware_version() == Packet::RJ2018) {
            batteryLevel = static_cast<float>(kRJ2015BatteryProfile.get_charge_level(rx.battery()));
        } else if (rx.hardware_version() == Packet::Simulation) {
            batteryLevel = 1;
        } else {
            std::cerr << "Unknown hardware revision " << rx.hardware_version()
                      << ", unable to calculate battery %" << std::endl;
        }
    }
    setBatteryLevel(batteryLevel);

    // If there is an error bad enough that we should get this robot
    // off the field, alert the user through the UI that there is a
    // "showstopper"
    bool showstopper = !hasVision || hasMotorFault || kickerFault || ballSenseFault ||
                       (batteryLevel < 0.25) || !fpgaWorking;
    setShowstopper(showstopper);
}
