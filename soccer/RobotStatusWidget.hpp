#pragma once

#include "ui_StatusWidget.h"
#include <QtWidgets>
#include <string>
#include <QtSvg>


/**
 * @brief Shows the status of a single robot
 * @details Includes things like battery voltage, radio connectivity, vision status, etc
 */
class RobotStatusWidget : public QWidget {
public:

	RobotStatusWidget(QWidget *parent = 0, Qt::WindowFlags f = 0);

	int shellID() const;
	void setShellID(int shellID);

	bool hasRadio() const;
	void setHasRadio(bool hasRadio);

	bool hasVision() const;
	void setHasVision(bool hasVision);

	float batteryLevel() const;
	void setBatteryLevel(float batteryLevel);


private:
	Ui_RobotStatusWidget _ui;

	int _shellID;
	bool _hasRadio;
	bool _hasVision;
	float _batteryLevel;
};
