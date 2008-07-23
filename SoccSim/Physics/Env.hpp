#ifndef ENV_HPP_
#define ENV_HPP_

#include <QThread>
#include <QVector>
#include <QMutex>

#include <ode/ode.h>

#include <Packet/CommData.hpp>
#include <Packet/SimCmd.hpp>
#include <Packet/PacketReceiver.hpp>
#include <Packet/PacketSender.hpp>
#include <Packet/RobotStatus.hpp>

#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

#include "Entity.hpp"

namespace Physics
{
	class Env : public QThread
	{
		private:
			class EnvReceiver : public QThread
			{
				public:
					/** make a new receiver that will control env */
					EnvReceiver(Env* env) : _env(env) {}
					
					void run();
					
				private:
					//simulation commands
					void simCmdHandler(const Packet::SimCmd* data);
					
					Env* _env;
			};
		
		//// methods ///
		public:
			Env();
			~Env();
			
			//list of all intities
			QVector<Entity*> entities() const { return _entities; }
			
			//get the ball
			Physics::Ball* ball() const { return _ball; }
			
			//get list of robots for the two teams
			QVector<Physics::Robot*> yellow();
			QVector<Physics::Robot*> blue();
			
			//add/remove robots
			void addRobot(Team t, unsigned int id);
			void removeRobot(Team t, unsigned int id);
			
			//enable/disable the ball
			void enableBall();
			void disableBall();
			
			//reset robots and ball to starting positions
			void resetRobot(Team t, unsigned int id);
			void resetBall();
			
			//pause if true, play if false
			void pause(bool p);
			
		protected:
			void run();
			
		private:
			//comm handlers for the teams
			static void commYellow(const Packet::CommData* data);
			static void commBlue(const Packet::CommData* data);
			
			//control a particular robot
			void controlRobot(Physics::Robot* r, const Packet::CommData::Robot& rData);
	
		private:
			//ode stuff
			dWorldID _world;
			dSpaceID _space;
			dJointGroupID _contactGroup;
			
			//everything
			QMutex _entitiesMutex;
			QVector<Entity*> _entities;
			
			//teams
			Physics::Robot* _yellow[5];
			Physics::Robot* _blue[5];
			
			//ball
			Physics::Ball* _ball;
			
			bool _running;
			
			//true if pause
			bool _paused;

			EnvReceiver _receiver;
			//receiver data store
			static Packet::CommData _blueData;
			static Packet::CommData _yellowData;
			
			//ode collision detection
			static void	nearCollision(void* _this, dGeomID id1, dGeomID id2);
	};
}

#endif /*ENV_HPP_*/
