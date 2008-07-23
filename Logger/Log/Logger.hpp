#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <QThread>
#include <QList>

#include "Loggable.hpp"

#include <Team.h>

namespace Log
{
	class Logger : public QThread
	{
		public:
			Logger(Team t);
			~Logger();
			
			QList<LogPacket*> dispPackets()
			{
				if (_loggable)
				{
					return _loggable->dispPackets();
				}
				else
				{
					return QList<LogPacket*>();
				}
			}
			
			/** return only the last display packet for each type */
			QList<LogPacket*> lastDispPackets()
			{
				if (_loggable)
				{
					return _loggable->lastDispPackets();
				}
				else
				{
					return QList<LogPacket*>();
				}
			}
			
			/** set the device to log to */
			void loggable(Log::Loggable* loggable)
			{
				_loggable = loggable;
				PacketType::loggable(_loggable);
			}
			
			/** return the types we are accepting */
			PacketType::TypeList types() const { return PacketType::types(); }
			
		protected:
			void run();
			
		private:
			bool _running;
			
			Team _team;
			
			/** the device we are logging to */
			static Log::Loggable* _loggable;
	};
}

#endif /*LOGGER_HPP_*/
