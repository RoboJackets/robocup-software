// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <boost/shared_ptr.hpp>
#include <QThread>
#include <QWaitCondition>
#include <QVector>
#include <RadioTx.hpp>

#include <Geometry2d/Point.hpp>
#include "Gamepad.hpp"

class InputHandler : public QThread
{
	Q_OBJECT;

	public:
		typedef boost::shared_ptr<InputHandler> shared_ptr;
		InputHandler(QObject* parent = 0);
		~InputHandler();
		
		void genRobotData(Packet::RadioTx::Robot &tx);

		/** returns true if a controller is active */
		bool enabled() const { return _running; }
		
	Q_SIGNALS:
		void playPauseButton();
		void manualAutoButton();
		void selectRobot(int rid);
		
	protected:
		void run();
		
	private:
		InputHandler(const InputHandler&);
		InputHandler& operator=(InputHandler&);

		bool _running;
		int8_t _roller;

		Gamepad* _controller;
		
		QWaitCondition _continueProcessing;

		QVector<Geometry2d::Point> _axles;
};
