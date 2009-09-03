#pragma once

#include <vector>
#include <Serialization.hpp>

namespace Network
{
	class Receiver
	{
		public:
			Receiver(uint16_t port, unsigned int size = 65536)
			{
				setup(0, port, size);
			}

			Receiver(const char* addr, uint16_t port, unsigned int size = 65536)
			{
				setup(addr, port, size);
			}

			~Receiver();
			
			int fileDescriptor() const { return _socket; }

			template<typename T>
			void receive(T &packet)
			{
				_buf.data.resize(_size);
				_buf.rewind();
				receive(_buf.data);

				Serialization::ReadBuffer &reader = _buf;
				reader & (T &) packet;
			}

			void receive(std::vector<uint8_t> &data);

		protected:
			void setup(const char *addr, uint16_t port, unsigned int size);

			int _socket;

			// Maximum packet size
			unsigned int _size;

			Serialization::MemoryBuffer _buf;
	};
}
