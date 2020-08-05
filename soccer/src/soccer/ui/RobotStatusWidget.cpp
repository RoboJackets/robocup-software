#include "RobotStatusWidget.hpp"

#include <cmath>
#include <iostream>

#include <BatteryProfile.hpp>
#include <rj_common/status.h>

RobotStatusWidget::RobotStatusWidget(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f) {
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

void RobotStatusWidget::setShellID(int shell_id) {
    if (shell_id != _shellID) {
        _shellID = shell_id;

        _ui.robotWidget->setShellID(_shellID);

        if (shell_id == -1) {
            _ui.shellID->setText(QString("?"));
        } else {
            _ui.shellID->setText(QString("%1").arg(shell_id));
        }
    }
}

void RobotStatusWidget::setErrorText(const QString& error) {
    _ui.errorText->setText(error);
}

void RobotStatusWidget::setBlueTeam(bool blue_team) {
    _ui.robotWidget->setBlueTeam(blue_team);
    _blueTeam = blue_team;
}

bool RobotStatusWidget::blueTeam() const { return _blueTeam; }

const QString& RobotStatusWidget::boardID() const { return _boardID; }

void RobotStatusWidget::setBoardID(const QString& board_id) {
    if (board_id != _boardID) {
        _boardID = board_id;
        _ui.boardID->setText(QString("ID: %1").arg(board_id));
    }
}

QString RobotStatusWidget::robotModel() const { return _ui.robotModel->text(); }

void RobotStatusWidget::setRobotModel(const QString& robot_model) {
    _ui.robotModel->setText(robot_model);
}

void RobotStatusWidget::setWheelFault(int wheel_index, bool faulty) {
    _ui.robotWidget->setWheelFault(wheel_index, faulty);
}

void RobotStatusWidget::setBallSenseFault(bool faulty) {
    _ui.robotWidget->setBallSenseFault(faulty);
}

void RobotStatusWidget::setHasBall(bool has_ball) {
    _ui.robotWidget->setHasBall(has_ball);
}

bool RobotStatusWidget::hasRadio() const { return _hasRadio; }

void RobotStatusWidget::setHasRadio(bool has_radio) {
    if (has_radio != _hasRadio) {
        _hasRadio = has_radio;

        _ui.radioIndicator->setPixmap(
            QPixmap(QString(has_radio ? ":icons/radio-connected.svg"
                                      : ":icons/radio-disconnected.svg")));
    }
}

bool RobotStatusWidget::hasVision() const { return _hasVision; }

void RobotStatusWidget::setHasVision(bool has_vision) {
    if (has_vision != _hasVision) {
        _hasVision = has_vision;

        _ui.visionIndicator->setPixmap(
            QPixmap(QString(has_vision ? ":icons/vision-available.svg"
                                       : ":icons/vision-unavailable.svg")));
    }
}

float RobotStatusWidget::batteryLevel() const { return _batteryLevel; }

void RobotStatusWidget::setBatteryLevel(float battery_level) {
    if (std::fabs(battery_level - _batteryLevel) > 0.01) {
        _batteryLevel = battery_level;
        _ui.batteryIndicator->setBatteryLevel(_batteryLevel);
    }
}

bool RobotStatusWidget::kickerState() const { return _kickerState; }

void RobotStatusWidget::setKickerState(bool kicker_state) {
    _kickerState = kicker_state;

    if (kicker_state) {
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

void RobotStatusWidget::loadFromLogFrame(
    const Packet::RadioRx& rx,
    const std::optional<Packet::LogFrame::Robot>& maybe_robot, bool blue_team) {
    // Set shell ID
    setShellID(rx.robot_id());

    // Set team
    setBlueTeam(blue_team);

    // set robot model
    QString robot_model;
    switch (rx.hardware_version()) {
        case Packet::HardwareVersion::RJ2008:
            robot_model = "RJ2008";
            break;
        case Packet::HardwareVersion::RJ2011:
            robot_model = "RJ2011";
            break;
        case Packet::HardwareVersion::RJ2015:
            robot_model = "RJ2015";
            break;
        case Packet::HardwareVersion::RJ2018:
            robot_model = "RJ2018";
            break;
        case Packet::HardwareVersion::Simulation:
            robot_model = "Simulation";
            break;
        default:
            robot_model = "Unknown Bot";
    }
    setRobotModel(robot_model);

    // Radio is always true if we're connected to it...
    setHasRadio(true);

    // vision status
    bool has_vision = maybe_robot.has_value();
    setHasVision(has_vision);

    // build a list of errors to display in the widget
    QStringList error_list;

    // Each motor fault is shown as text in the error text display
    // as well as being drawn as a red X on the graphic of a robot
    bool has_motor_fault = false;
    if (rx.motor_status().size() == 5) {
        std::array<const char*, 5> motor_names = {"FL", "BL", "BR", "FR",
                                                  "Dribbler"};

        // Examine status of each motor (including the dribbler)
        for (int i = 0; i < 5; ++i) {
            bool motor_fault = rx.motor_status(i) != Packet::MotorStatus::Good;

            if (motor_fault) {
                error_list << QString("Motor Fault %1").arg(motor_names[i]);
                motor_fault = true;
            }

            if (i != 4) {
                // Show wheel faults (exluding dribbler, which is index 4)
                setWheelFault(i, motor_fault);
            } else {
                // Dribbler fault shows up as ball sense fault.
                setBallSenseFault(motor_fault);
            }

            has_motor_fault = has_motor_fault || motor_fault;
        }
    } else {
        std::cerr << "Expected 5 motors, but only " << rx.motor_status_size()
                  << std::endl;
        has_motor_fault = true;
    }

    // Kicker
    bool kicker_fault = true;
    bool kicker_charging = false;
    if (rx.has_kicker_status()) {
        kicker_fault = (rx.kicker_status() & Kicker_Enabled) == 0u;
        kicker_charging = (rx.kicker_status() & Kicker_Charged) == 0u;
    }
    if (kicker_fault) {
        error_list << "Kicker Fault";
    }
    setKickerState(kicker_charging);

    // Ball sense
    bool ball_sense_fault = true;
    if (rx.has_ball_sense_status()) {
        ball_sense_fault = !(rx.ball_sense_status() == Packet::NoBall ||
                             rx.ball_sense_status() == Packet::HasBall);
    }
    if (ball_sense_fault) {
        error_list << "Ball Sense Fault";
    }
    setBallSenseFault(ball_sense_fault);

    // FPGA
    bool fpga_working = true;
    if (rx.has_fpga_status() && rx.fpga_status() != Packet::FpgaGood) {
        if (rx.fpga_status() == Packet::FpgaNotInitialized) {
            error_list << "FPGA not initialized";
        } else {
            error_list << "FPGA error";
        }
        fpga_working = false;
    }

    // Display error text
    setErrorText(error_list.join(", "));

    // Show the ball in the robot's mouth if it has one
    bool has_ball =
        rx.has_ball_sense_status() && rx.ball_sense_status() == Packet::HasBall;
    setHasBall(has_ball);

    // Convert battery voltage to a percentage and show it with the battery
    // indicator
    float battery_level = 1;
    if (rx.has_battery()) {
        if (rx.hardware_version() == Packet::RJ2008 ||
            rx.hardware_version() == Packet::RJ2011) {
            battery_level = static_cast<float>(
                RJ2008BatteryProfile.getChargeLevel(rx.battery()));
        } else if (rx.hardware_version() == Packet::RJ2015 ||
                   rx.hardware_version() == Packet::RJ2018) {
            battery_level = static_cast<float>(
                RJ2015BatteryProfile.getChargeLevel(rx.battery()));
        } else if (rx.hardware_version() == Packet::Simulation) {
            battery_level = 1;
        } else {
            std::cerr << "Unknown hardware revision " << rx.hardware_version()
                      << ", unable to calculate battery %" << std::endl;
        }
    }
    setBatteryLevel(battery_level);

    // If there is an error bad enough that we should get this robot
    // off the field, alert the user through the UI that there is a
    // "showstopper"
    bool showstopper = !has_vision || has_motor_fault || kicker_fault ||
                       ball_sense_fault || (battery_level < 0.25) ||
                       !fpga_working;
    setShowstopper(showstopper);
}
