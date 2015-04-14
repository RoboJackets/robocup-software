#pragma once

#include <QtWidgets>


/**
 * @brief Presents an image of a robot with errors drawn in red
 */
class RobotWidget : public QWidget {
public:
    RobotWidget(QWidget *parent = 0, Qt::WindowFlags f = 0);

    void setBlueTeam(bool blueTeam);
    bool blueTeam() const;
    void setShellID(int shellID);

    void setWheelFault(int wheelIndex, bool faulty = true);
    void setBallSenseFault(bool faulty = true);
    void setHasBall(bool hasBall = true);
    
    void paintEvent(QPaintEvent *event);


private:
    bool _wheelFaults[4];
    bool _ballSenseFault;
    bool _hasBall;
    int _shellID;
    bool _blueTeam;
};
