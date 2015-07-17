#pragma once

#include <QtWidgets>


/**
 * @brief Draws a battery given a battery level value from 0 to 1
 */
class BatteryWidget : public QWidget {
public:
	BatteryWidget (QWidget *parent = 0, Qt::WindowFlags f = 0);

	/**
	 * @brief Battery level represented by a number between 0 and 1
	 */
	float batteryLevel() const;
	void setBatteryLevel(float batteryLevel);


	void paintEvent(QPaintEvent *event);


private:
	float _batteryLevel;
};
