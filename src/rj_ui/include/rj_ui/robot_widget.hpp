#pragma once

#include <cmath>
#include <stdexcept>

#include <QtWidgets>

#include <rj_common/vision_dot_pattern.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/util.hpp>

/**
 * @brief Presents an image of a robot with errors drawn in red.  A ball is
 * drawn in front of it if it has it.
 */
class RobotWidget : public QWidget {
public:
    RobotWidget(QWidget* parent = nullptr, Qt::WindowFlags f = {});

    void setBlueTeam(bool blueTeam);
    bool blueTeam() const;
    void setShellID(int shellID);

    void setWheelFault(int wheelIndex, bool faulty = true);
    void setBallSenseFault(bool faulty = true);
    void setHasBall(bool hasBall = true);

    void paintEvent(QPaintEvent* event) override;

private:
    bool _wheelFaults[4]{};
    bool _ballSenseFault;
    bool _hasBall;
    int _shellID;
    bool _blueTeam;
};
