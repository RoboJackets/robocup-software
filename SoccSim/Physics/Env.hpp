#ifndef _ENV_HPP
#define _ENV_HPP

#include <NxPhysics.h>

#include <QVector>
#include <QTimer>
#include <QMutex>
#include <Team.h>
#include <Geometry/Point2d.hpp>
#include <Vision.hpp>
#include <RadioTx.hpp>
#include <RadioRx.hpp>

#include "Ball.hpp"
#include "Robot.hpp"
#include "Field.hpp"

class Env : public QObject
{
	Q_OBJECT;

	public:
		Env();
		~Env();

		/** return the debug renderable for the environment */
		const NxDebugRenderable& dbgRenderable() const;

		void start();

		/** add a ball @ pos */
		void addBall(Geometry::Point2d pos);
		
		/** add a robot with id i to the environment @ pos */
		void addRobot(Team t, int id, Geometry::Point2d pos);
		
		/** @return latest vision information about the environment */
		Packet::Vision vision();
		
		/** @return generated return data from robots */
		Packet::RadioRx radio(Team t);
		
		/** set received robot control data, used next loop cycle */
		void radio(Team t, Packet::RadioTx& data);
		
		//const QVector<const Robot*>& getRobots(Team t);
		//const QVector<const Ball*>& getBalls();

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
};

#endif /* _ENV_HPP */
