#ifndef ROBOTSTATUS_HPP_
#define ROBOTSTATUS_HPP_

#include "../Packet.hpp"
#include <Packet/RobotStatus.hpp>

class QProgressBar;
class QCheckBox;
class QLabel;

namespace Log
{
	class RobotStatusType : public PacketType
	{
		public:
			class RobotStatus : public LogPacket
			{
				public:
					RobotStatus(Packet::RobotStatus data) :
						LogPacket(data.timestamp), _data(data) {}
					
					virtual LogPacket* clone() const
					{
						return new RobotStatus(*this);
					}

					virtual PacketType* type() const { return &_type; }

					virtual void updateInformationWidget();
					
				private:
					Packet::RobotStatus _data;
					static Log::RobotStatusType _type;
			};
			
			RobotStatusType() : PacketType(Packet::RobotStatus::Type, "Robot Status") {}

			static void handle(const Packet::RobotStatus* data)
			{
				if (data)
				{
					PacketType::log(new RobotStatus(*data));
				}
			}

			//TODO fixme, make the receiver store the team if we want
			virtual void listen(Packet::PacketReceiver& receiver) const
			{
				receiver.addType( handle);
			}
			
			//virtual QWidget* configuration();
			virtual QWidget* information();
			
		private:
			QProgressBar* _batteries[5];
			QLabel* _voltages[5];
			QProgressBar* _rssi[5];
			QCheckBox* _charged[5];
			QCheckBox* _ball[5];
	};
}

#endif /* ROBOTSTATUS_HPP_ */
