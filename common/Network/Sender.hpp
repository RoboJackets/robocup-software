#pragma once

#include <netinet/in.h>
#include <vector>

#include <Serialization.hpp>

namespace Network
{
	class Sender
	{
		public:
			Sender(const char *addr, uint16_t port);
			~Sender();

			template<typename T>
			void send(const T &packet)
			{
				// Erase data but keep capacity to avoid reallocation.
				_buf.data.resize(0);

				Serialization::WriteBuffer &writer = _buf;
				writer & (T &) packet;
				send(&_buf.data[0], _buf.data.size());
			}

			void send(const uint8_t *data, unsigned int len);

		protected:
			int _socket;
			struct sockaddr_in _dest;

			Serialization::MemoryBuffer _buf;
	};
}
