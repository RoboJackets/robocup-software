#ifndef IO_HPP
#define IO_HPP

#include "UdpSocket.hpp"

#include <QHostAddress>

#include <stdexcept>
#include <errno.h>

#include <Team.h>

#define YellowBase  30000
#define BlueBase    20000
#define RefBoxPort  10001

//address for common items
#define CommonAddr    QHostAddress("226.0.0.1")

//Refbox address
#define RefBoxAddr    QHostAddress("224.5.23.1")

namespace Packet
{
	/** A Sender for sending to a multicast address and udp socket */
	template <typename T>
	class Sender : public UdpSocket
	{
		public:
			Sender(Team t) :
				UdpSocket(), _addr(CommonAddr), _port(T::Type)
			{
				if (t == Yellow)
				{
					_port += YellowBase;
				}
				else
				{
					_port += BlueBase;
				}
			};

			~Sender()
			{
			};

			void send(const T outData)
			{
				if (writeDatagram((const char*)&outData, sizeof(T), _addr,
						_port) != sizeof(T))
				{
					throw std::runtime_error(strerror(errno));
				}
			}

		private:
			QHostAddress _addr;
			uint16_t _port;
	};


	/** Receive from a multicast address and udp socket */
	template <typename T>
	class Receiver : public UdpSocket
	{
		public:
			Receiver(QHostAddress addr, uint16_t port) :
				UdpSocket()
			{
				if (!bind(port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
				{
					throw std::runtime_error(strerror(errno));
				}

				if (!join(addr))
				{
					throw std::runtime_error("Unable to Join multicast group");
				}
			};

			~Receiver()
			{
			};

			bool read(T& inData)
			{
				//return readDatagram((char*)&inData, sizeof(T)) == sizeof(T);
				const int i = readDatagram((char*)&inData, sizeof(T));
				
				//printf("%d %d\n", i, sizeof(T));
				if (i == -1)
				{
					throw std::runtime_error(strerror(errno));
				}
				
				if (i != sizeof(T))
				{
					char err[200];
					snprintf(err, sizeof(err), "Received packet size %d, expected %u.", i, (unsigned int)sizeof(T));
					throw std::runtime_error(err);
				}

				return true;
			}

			uint16_t port() const
			{
				return localPort();
			}
	};
}

#endif /* IO_HPP */
