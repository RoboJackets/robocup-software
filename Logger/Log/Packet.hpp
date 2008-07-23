#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <QPainter>

#include <Packet/PacketReceiver.hpp>

namespace Log
{	
	class LogPacket;
	class Loggable;
	class PacketType
	{		
		public:
			typedef QList<PacketType*> TypeList;
			
		public:
			PacketType(uint16_t id, const char* name);
			
			virtual QWidget* configuration() { return 0; }
			virtual QWidget* information() { return 0; }
			
			static TypeList types() { return *_types; }
	
			uint16_t id() const { return _id; }
			
			const char* name() const { return _name; }
			
			/** add the type to the packet receiver */
			virtual void listen(Packet::PacketReceiver& receiver) const = 0;
			static void log(Log::LogPacket* p);
			static void loggable(Log::Loggable* loggable) { _loggable = loggable; }
			
		protected:
			PacketType();
			
		private:
			static TypeList* _types;
			
			uint16_t _id;
			
			const char* _name;
			
			static Log::Loggable* _loggable;
	};
	
	/** general packet that can be display, configured, etc... */
	class LogPacket
	{
		public:
			LogPacket(uint64_t id) : _id(id) {}
			virtual ~LogPacket() {};
			
			virtual LogPacket* clone() const = 0;
			virtual PacketType* type() const = 0;
			
			//updates the information widget
			virtual void updateInformationWidget() {};
			
			virtual void display(QPainter& p, Team t) {};
			
			uint64_t id() const { return _id; }
		protected:
			/** id = timestamp, unique for a packet type */
			const uint64_t _id;
	};
}

#endif /*PACKET_HPP_*/
