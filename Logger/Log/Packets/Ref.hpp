#ifndef COMMDATA_HPP_
#define COMMDATA_HPP_

#include "../Packet.hpp"
#include <Packet/Ref.hpp>

class QLabel;

namespace Log
{
	class RefType : public PacketType
	{
		public:
			//vision data packet wrapper
			class Ref : public LogPacket
			{
				public:
					Ref(Packet::Ref data) :
						LogPacket(data.timestamp), _data(data) {}

					virtual LogPacket* clone() const
					{
						return new Ref(*this);
					}

					virtual PacketType* type() const { return &_type; }

					virtual void updateInformationWidget();

				private:
					Packet::Ref _data;
					static Log::RefType _type;
			};

			RefType() : PacketType(Packet::Ref::Type, "Ref") {}

			static void handle(const Packet::Ref* data)
			{
				if (data)
				{
					PacketType::log(new Ref(*data));
				}
			}

			//TODO fixme, make the receiver store the team if we want
			virtual void listen(Packet::PacketReceiver& receiver) const
			{
				receiver.addType(handle);
			}

			//virtual QWidget* configuration();
			virtual QWidget* information();

		private:
			QLabel* _state;
			QLabel* _start;
			QLabel* _period;
			QLabel* _ourStart;
			
	};
}

#endif /*COMMDATA_HPP_*/
