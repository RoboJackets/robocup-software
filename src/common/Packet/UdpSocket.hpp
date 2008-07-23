#ifndef UDPSOCKET_HPP_
#define UDPSOCKET_HPP_

#include <QUdpSocket>

#include <stdint.h>

class QHostAddress;

namespace Packet
{
	class UdpSocket : public QUdpSocket
	{
		public:
			UdpSocket();
			
			/** join the multicast group of the address */
			bool join(QHostAddress addr);
			
		private:
			UdpSocket(UdpSocket&);
			UdpSocket& operator=(UdpSocket&);
	};
}

#endif /*UDPSOCKET_HPP_*/
