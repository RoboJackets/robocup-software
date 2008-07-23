#ifndef LOGMOTION_HPP_
#define LOGMOTION_HPP_

#include "../Packet.hpp"

#include <Packet/LogMotion.hpp>

namespace Log
{
	class LogMotionType : public PacketType
	{
		public:
			//vision data packet wrapper
			class LogMotion : public LogPacket
			{
				public:
					LogMotion(Packet::LogMotion data) : 
						LogPacket(data.timestamp), _data(data) {}
					
					virtual LogPacket* clone() const
					{
						return new LogMotion(*this);
					}
					
					virtual PacketType* type() const { return &_type; }
					
					virtual void display(QPainter& p, Team t);
				private:
					Packet::LogMotion _data;	
					static Log::LogMotionType _type;
			};
			
			LogMotionType() : PacketType(Packet::LogMotion::Type, "Log Motion") {}
			
			static void handle(const Packet::LogMotion* data)
			{
				if (data)
				{
					PacketType::log(new LogMotion(*data));
				}
			}
			
			//TODO fixme, make the receiver store the team if we want
			virtual void listen(Packet::PacketReceiver& receiver) const
			{
				receiver.addType(handle);
			}
			
			virtual QWidget* configuration();
			
		private:
	};
}

#endif /*LOGMOTION_HPP_*/
