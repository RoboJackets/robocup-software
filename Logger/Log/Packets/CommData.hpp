#ifndef COMMDATA_HPP_
#define COMMDATA_HPP_

#include "../Packet.hpp"
#include <Packet/CommData.hpp>

class QProgressBar;

namespace Log
{
	class CommDataType : public PacketType
	{
		public:
			//vision data packet wrapper
			class CommData : public LogPacket
			{
				public:
					CommData(Packet::CommData data) :
						LogPacket(data.timestamp), _data(data) {}

					virtual LogPacket* clone() const
					{
						return new CommData(*this);
					}

					virtual PacketType* type() const { return &_type; }

					virtual void updateInformationWidget();

				private:
					Packet::CommData _data;
					static Log::CommDataType _type;
			};

			CommDataType() : PacketType(Packet::CommData::Type, "Comm Data") {}

			static void handle(const Packet::CommData* data)
			{
				if (data)
				{
					PacketType::log(new CommData(*data));
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
			//information widget stuff
			QProgressBar* _motors[5][4];
	};
}

#endif /*COMMDATA_HPP_*/
