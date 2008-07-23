#include "Packet.hpp"

#include "Loggable.hpp"

using namespace Log;

Log::PacketType::TypeList* PacketType::_types;
Log::Loggable* PacketType::_loggable = 0;

PacketType::PacketType(uint16_t id, const char* name) : _id(id), _name(name)
{
	if (!_types)
	{
		_types = new TypeList();
	}
	
	printf("Accepting type: %s\n", this->name());
	_types->append(this);
}

void PacketType::log(LogPacket* p)
{
	if (_loggable)
	{
		_loggable->log(p);
	}
}
