#pragma once

#include "ui_RobotStatusWidget.h"
#include <QtWidgets>
#include <string>


/**
 * @brief Shows the status of a single robot
 * @details Includes things like battery voltage, radio connectivity, vision status, etc
 */
class RobotStatusWidget : public QWidget {
public:

	RobotStatusWidget(QWidget *parent = nullptr, Qt::WindowFlags f = nullptr);

	int shellID() const;
	void setShellID(int shellID);

	void setBlueTeam(bool blueTeam);
	bool blueTeam() const;

	/**
	 * @brief This error text is displayed in a text box
	 */
	void setErrorText(const QString &error);

	/**
	 * @brief ID of control board (4 hex digits)
	 */
	const QString &boardID() const;
	void setBoardID(const QString &boardID);


	void setWheelFault(int wheelIndex, bool faulty = true);
    void setBallSenseFault(bool faulty = true);
    void setHasBall(bool hasBall = true);


	/**
	 * @brief ID of mechanical base (4 hex digits)
	 */
	const QString &baseID() const;
	void setBaseID(const QString &baseID);

	/**
	 * @brief A string describing the robot model (i.e. "RJ 2015")
	 */
	QString robotModel() const;
	void setRobotModel(const QString &robotModel);

	bool hasRadio() const;
	void setHasRadio(bool hasRadio);

	bool hasVision() const;
	void setHasVision(bool hasVision);

	float batteryLevel() const;
	void setBatteryLevel(float batteryLevel);

	/**
	 * @brief Set to true to present this robot as being in critical
	 * status and in need of removing from the field.
	 */
	void setShowstopper(bool showstopper = true);


private:
	Ui_RobotStatusWidget _ui;

	int _shellID;
	bool _blueTeam;
	bool _hasRadio;
	bool _hasVision;
	float _batteryLevel;
	bool _showstopper;
	QString _boardID;
};
