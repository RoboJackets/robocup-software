#ifndef _VISION__SENDER_H_
#define _VISION__SENDER_H_

#include "Transform.h"
#include "Tracker.h"

#include <Geometry/TransformMatrix.hpp>

#include <QMutex>
#include <QHostAddress>

#include <netinet/in.h>

namespace Vision
{
	class Process;
	
	class Sender
	{
	public:
		Sender();
		
		// Total width of the field in meters, used to calculate team space transformations
		void field_width(float w);
		
		void update(Vision::Process *process);
		
	protected:
		class Object_Data
		{
		public:
			Object_Data()
			{
				owner = 0;
			}
			
			Process *owner;
			Track track;
		};
		
		class Team_Data
		{
		public:
			Team_Data()
			{
				socket = -1;
			}
			
			int socket;
            struct sockaddr_in sin;

			// Transformations into team space
			Geometry::TransformMatrix transform;
			
			Object_Data robot[5];
		};
		
		QMutex mutex;
		
		Team_Data yellow, blue;
		
		Object_Data _ball;
		
		// The process that last reported each object.
		Process *_ball_process;
		
		void create_socket(Team_Data &team, QHostAddress addr, uint16_t port);
		void send_team(const Team_Data &self, const Team_Data &opp);
		void update_object(Object_Data &object, Process *process, const Track *track);
	};
};

#endif // _VISION__SENDER_H_
