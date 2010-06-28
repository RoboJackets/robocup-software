#pragma once

#include <NxPhysics.h>

#include <QVector>
#include <QMap>
#include <QTimer>
#include <QMutex>
#include <QUdpSocket>
#include <sys/time.h>

#include <Team.h>
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

		/** return the debug renderable for the environment */
		NxDebugRenderable dbgRenderable() const;

		/** add a ball @ pos */
		void addBall(Geometry2d::Point pos);
		
		/** add a robot with id i to the environment @ pos */
		void addRobot(Team t, int id, Geometry2d::Point pos, Robot::Rev rev);
		
		/** @return latest vision information about the environment */
// 		Packet::Vision vision();
		
		/** @return generated return data from robots */
// 		Packet::RadioRx radioRx(Team t);
		
		/** set received robot control data, used next loop cycle */
// 		void radioTx(Team t, const Packet::RadioTx& data);
		
//         void command(const Packet::SimCommand &cmd);
        
        const QVector<Ball*> &balls() const
        {
            return _balls;
        }
        
		NxPhysicsSDK* _physicsSDK;

	protected Q_SLOTS:
		void step();
		
	private:
		Robot *robot(Team t, int board_id) const;
		static void convert_robot(const Robot *robot, SSL_DetectionRobot *out);

// 		mutable QMutex _sceneMutex;
		NxScene* _scene;
		
		Field* _field;
		
// 		mutable QMutex _entitiesMutex;
		QMap<unsigned int, Robot*> _blue;
		QMap<unsigned int, Robot*> _yellow;
		QVector<Ball*> _balls;
		
		// This timer causes physics to be stepped on a regular basis
		QTimer _timer;
		
		QUdpSocket _visionSocket;
		int _frameNumber;
		
		struct timeval _lastStepTime;
		
		// How many physics steps have run since the last vision packet was sent
		int _stepCount;
		
// 		QMutex _visionMutex;
// 		SSL_WrapperPacket _vision;
		
// 		QMutex _radioRxMutex;
// 		Packet::RadioRx _radioRxBlue;
// 		Packet::RadioRx _radioRxYellow;
		
		// Returns true if any robot occludes a ball from a camera's point of view.
		bool occluded(Geometry2d::Point ball, Geometry2d::Point camera);
};
