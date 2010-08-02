#pragma once

#include <NxPhysics.h>

#include <QVector>
#include <QMap>
#include <QTimer>
#include <QMutex>
#include <QUdpSocket>
#include <sys/time.h>

#include <Geometry2d/Point.hpp>

#include "Ball.hpp"
#include "Robot.hpp"
#include "Field.hpp"

class SSL_DetectionRobot;

class Env : public QObject
{
	Q_OBJECT;
	
	public:
		Env();
		~Env();
        
		NxScene* scene() const
		{
			return _scene;
		}

		const QVector<Ball*> &balls() const
		{
			return _balls;
		}

		/** return the debug renderable for the environment */
		NxDebugRenderable dbgRenderable() const;

		/** add a ball @ pos */
		void addBall(Geometry2d::Point pos);
		
		/** add a robot with id i to the environment @ pos */
		void addRobot(bool blue, int id, Geometry2d::Point pos, Robot::Rev rev);
		
		// If true, send data to the shared vision multicast address.
		// If false, send data to the two simulated vision addresses.
		volatile bool sendShared;
		
		NxPhysicsSDK* _physicsSDK;

	protected Q_SLOTS:
		void step();
		
	private:
		Robot *robot(bool blue, int board_id) const;
		static void convert_robot(const Robot *robot, SSL_DetectionRobot *out);
		void handleRadioTx(int ch, const Packet::RadioTx& data);

		NxScene* _scene;
		
		Field* _field;
		
		typedef QMap<unsigned int, Robot*> RobotMap;
		RobotMap _blue;
		RobotMap _yellow;
		QVector<Ball*> _balls;
		
		// This timer causes physics to be stepped on a regular basis
		QTimer _timer;
		
		QUdpSocket _visionSocket;
		QUdpSocket _radioSocket[2];
		
		struct timeval _lastStepTime;
		
		// How many vision frames we've sent
		int _frameNumber;
		
		// How many physics steps have run since the last vision packet was sent
		int _stepCount;
		
		// Returns true if any robot occludes a ball from a camera's point of view.
		bool occluded(Geometry2d::Point ball, Geometry2d::Point camera);
};
