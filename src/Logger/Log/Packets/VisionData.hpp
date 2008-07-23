#ifndef VISIONDATA_HPP_
#define VISIONDATA_HPP_

#include "../Packet.hpp"

#include <Packet/VisionData.hpp>

class QWidget;
class QLabel;

namespace Log
{
	class VisionDataType : public PacketType
	{
		public:
			//vision data packet wrapper
			class VisionData : public LogPacket
			{
				public:
					VisionData(Packet::VisionData data) : 
						LogPacket(data.timestamp), _data(data) {}
					
					virtual LogPacket* clone() const
					{
						return new VisionData(*this);
					}
					
					virtual PacketType* type() const { return &_type; }
					
					Packet::VisionData data() const { return _data; }
					
					virtual void updateInformationWidget();
					virtual void display(QPainter& p, Team t);
					
				private:
					Packet::VisionData _data;
					static Log::VisionDataType _type;
			};
			
			VisionDataType() : PacketType(Packet::VisionData::Type, "Vision Data") {}
			
			static void handle(const Packet::VisionData* data)
			{
				if (data)
				{
					PacketType::log(new VisionData(*data));
				}
			}
			
			//TODO fixme, make the receiver store the team if we want
			virtual void listen(Packet::PacketReceiver& receiver) const
			{
				receiver.addType(handle);
			}
			
			virtual QWidget* configuration();
			virtual QWidget* information();
			
		private:
			//information widget stuff
			QWidget* _infoWidget;
			QLabel* _timestamp;
			QLabel* _robots[10];
	};
}

#endif /*VISIONDATA_HPP_*/
