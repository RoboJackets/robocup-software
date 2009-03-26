#ifndef INPUTHANDLER_HPP_
#define INPUTHANDLER_HPP_

#include <QThread>
#include <QWaitCondition>
#include <QVector>
#include <RadioTx.hpp>

#include <Geometry/Point2d.hpp>
#include "Gamepad.hpp"

class InputHandler : public QThread
{
	Q_OBJECT;

	public:
		InputHandler(QObject* parent = 0);
		~InputHandler();
		
		Packet::RadioTx::Robot genRobotData();

		/** returns the currently selected robot */
		unsigned int currentRobot() const { return _rid; }
        
        /** returns true if a controller is active */
        bool enabled() const { return _running; }
		
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
		
		QWaitCondition _continueProcessing;

		/** last selected robot for manual control */
		unsigned int _rid;
		
		QVector<Geometry::Point2d> _axels;
};

#endif /*INPUTHANDLER_HPP_*/
