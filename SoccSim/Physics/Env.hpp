#pragma once

#include <NxPhysics.h>

#include <QVector>
#include <QTimer>
#include <QMutex>
#include <Team.h>
#include <Geometry2d/Point.hpp>
#include <Vision.hpp>
#include <RadioTx.hpp>
#include <RadioRx.hpp>
#include <SimCommand.hpp>

#include "Ball.hpp"
#include "Robot.hpp"
#include "Field.hpp"

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
		const NxDebugRenderable& dbgRenderable() const;

		void start();

		/** add a ball @ pos */
		void addBall(Geometry2d::Point pos);
		
		/** add a robot with id i to the environment @ pos */
		void addRobot(Team t, int id, Geometry2d::Point pos);
		
		/** @return latest vision information about the environment */
		Packet::Vision vision();
		
		/** @return generated return data from robots */
		Packet::RadioRx radioRx(Team t);
		
		/** set received robot control data, used next loop cycle */
		void radioTx(Team t, const Packet::RadioTx& data);
		
        void command(const Packet::SimCommand &cmd);
        
        const QVector<Ball*> &balls() const
        {
            return _balls;
        }
        
	private Q_SLOTS:
		void step();
		
		void genVision();
		void genRadio();

	public:
		/** the global physics sdk for every environment */
		static NxPhysicsSDK* _physicsSDK;

	private:
		/** number of open environments */
		static unsigned int _refCount;

		Robot *robot(Team t, int board_id) const;

		mutable QMutex _sceneMutex;
		/** local scene for this environment */
		NxScene* _scene;
		
		Field* _field;
		
		mutable QMutex _entitiesMutex;
		QVector<Robot*> _blue;
		QVector<Robot*> _yellow;
		QVector<Ball*> _balls;
		
		QTimer _step;
		
		QMutex _visionMutex;
		Packet::Vision _visionInfo;
		
		QMutex _radioRxMutex;
		Packet::RadioRx _radioRxBlue;
		Packet::RadioRx _radioRxYellow;
		
		// Returns true if any robot occludes a ball from a camera's point of view.
		bool occluded(Geometry2d::Point ball, Geometry2d::Point camera);
};
