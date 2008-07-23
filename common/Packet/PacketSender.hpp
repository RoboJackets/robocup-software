#ifndef PACKETSENDER_HPP_
#define PACKETSENDER_HPP_

#include <QUdpSocket>
#include <Team.h>

#include "IO.hpp"

namespace Packet
{
	class PacketSender : private QUdpSocket
	{
		public:
			PacketSender(Team t) : QUdpSocket(), _addr(CommonAddr), _team(t) {}
			~PacketSender() {}

			template <typename T>
			void send(const T outData)
			{
				uint16_t port = T::Type;
				port += ((_team == Yellow) ? YellowBase : BlueBase);
				
				if (writeDatagram((const char*)&outData, sizeof(T), _addr,
						port) != sizeof(T))
				{
					throw std::runtime_error(strerror(errno));
				}
			}

		private:
			QHostAddress _addr;
			
			Team _team;
	};
}

#endif /* PACKETSENDER_HPP_ */
