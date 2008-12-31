#ifndef _ENV_HPP
#define _ENV_HPP

#include <NxPhysics.h>

#include <QVector>
#include <QTimer>
#include <Team.h>
#include <Network/PacketReceiver.hpp>
#include <Network/Sender.hpp>
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

	private Q_SLOTS:
		void step();

        protected:
                void radioHandler(const Packet::RadioTx* packet);

	public:
		/** the global physics sdk for every environment */
		static NxPhysicsSDK* _physicsSDK;

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

                /** Reciever for radio commands. Temporary until something better comes along**/
                Network::PacketReceiver* _receiver;

                const Packet::RadioTx* _txPacket;

};

#endif /* _ENV_HPP */
