#ifndef _ENV_HPP
#define _ENV_HPP

#include <NxPhysics.h>

#include <QVector>
#include <QTimer>
#include <Team.h>
#include <Geometry/Point2d.hpp>
#include <RadioTx.hpp>

#include "../InputHandler.hpp"

class Entity;
class Robot;

class Env : public QObject
{
	Q_OBJECT;

	public:
		Env();
		~Env();

		/** return the debug renderable for the environment */
		const NxDebugRenderable& dbgRenderable() const;

		void start();

		void addBall(float x, float y);
		void addRobot(int id, float x, float y);

                QVector<Geometry::Point2d*> getRobotsPositions();

                QVector<Geometry::Point2d*> getBallPositions();

	private Q_SLOTS:
		void step();

	public:
		/** the global physics sdk for every environment */
		static NxPhysicsSDK* _physicsSDK;

                const Packet::RadioTx* txPacket;

	private:
		/** number of open environments */
		static unsigned int _refCount;

		/** local scene for this environment */
		NxScene* _scene;

		/** all of the created entities for the environment */
		QVector<Entity*> _entities;

		//temp
		QVector<Robot*> _robots;

		QTimer _step;

		InputHandler* inputHandler;
};

#endif /* _ENV_HPP */
