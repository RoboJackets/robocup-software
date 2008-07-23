#ifndef INPUTHANDLER_HPP_
#define INPUTHANDLER_HPP_

#include <QThread>
#include <QWaitCondition>
#include <QVector>

#include <Packet/CommData.hpp>
#include <Geometry/Point2d.hpp>

#include "ConfigFile.hpp"
#include "Gamepad.hpp"

class InputHandler : public QThread
{
	Q_OBJECT;
	
	public:
		InputHandler(ConfigFile& cfg);
		~InputHandler();
		
		Packet::CommData::Robot genRobotData();
		
		/** returns the currently selected robot */
		unsigned int currentRobot() const { return _rid; }
		
	Q_SIGNALS:
		void playPauseButton();
		void manualAutoButton();
		void changeRobot(int rid);
		
	protected:
		void run();
		
	private:
		InputHandler(const InputHandler&);
		InputHandler& operator=(InputHandler&);
		
		bool _running;
		int8_t _roller;
        
		Gamepad* _controller;
		
		JoystickInput* _temp;
		
		QWaitCondition _continueProcessing;
		
		/** last selected robot for manual control */
		unsigned int _rid;
		
		QVector<Geometry::Point2d> _axels;
};

#endif /*INPUTHANDLER_HPP_*/
