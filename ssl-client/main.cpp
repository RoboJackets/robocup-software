#include <stdio.h>

#include <messages_robocup_ssl_detection.pb.h>
#include <messages_robocup_ssl_wrapper.pb.h>
#include <messages_robocup_ssl_geometry.pb.h>

#include <vector>
#include <Network/Receiver.hpp>
#include <Network/Sender.hpp>
#include <Network/Network.hpp>
#include <Geometry2d/util.h>

#include <boost/foreach.hpp>

#include <Point.hpp>
#include <Vision.hpp>

#include <Utils.hpp>

using namespace std;

int main(int argc, char* argv[])
{
	Network::Receiver sslReceiver("224.5.23.2", 10002);
	Network::Sender rjSender(Network::Address, Network::Vision);

	SSL_WrapperPacket packet;
	SSL_DetectionFrame detection;
	vector<uint8_t> buffer;

	///loop
	while (true)
	{
		buffer.resize(65536);

		sslReceiver.receive(buffer);
		bool protoStatus = packet.ParseFromArray(&buffer[0], buffer.size());

		if (protoStatus && packet.has_detection())
		{
			SSL_DetectionFrame detection = packet.detection();
			
#if 0
			if (detection.camera_id() == 0)
			{
				continue;
			}
#endif		
			printf("Camera: %d\n", detection.camera_id());
			
			/// sync frame
			Packet::Vision sync;
			sync.timestamp = Utils::timestamp();
			sync.camera = detection.camera_id();
			sync.sync = true;
			rjSender.send(sync);
			
			/// regular frame 

			Packet::Vision visionPacket;
			visionPacket.camera = detection.camera_id();
			visionPacket.timestamp = Utils::timestamp();
			
			BOOST_FOREACH(const SSL_DetectionRobot& robot, detection.robots_yellow())
			{
				if (robot.confidence() == 0)
				{
					continue;
				}

				Packet::Vision::Robot r;
				r.pos.x = robot.x()/1000.0;
				r.pos.y = robot.y()/1000.0;
				r.angle = robot.orientation() * RadiansToDegrees;
				r.shell = robot.robot_id();
				
				visionPacket.yellow.push_back(r);
			}

			BOOST_FOREACH(const SSL_DetectionRobot& robot, detection.robots_blue())
			{
				if (robot.confidence() == 0)
				{
					continue;
				}
				
				Packet::Vision::Robot r;
				r.pos.x = robot.x()/1000.0;
				r.pos.y = robot.y()/1000.0;
				r.angle = robot.orientation() * RadiansToDegrees;
				r.shell = robot.robot_id();
				
				visionPacket.blue.push_back(r);
			}
			
			BOOST_FOREACH(const SSL_DetectionBall& ball, detection.balls())
			{
				if (ball.confidence() == 0)
				{
					continue;
				}

				Packet::Vision::Ball b;
				b.pos.x = ball.x()/1000.0;
				b.pos.y = ball.y()/1000.0;
				visionPacket.balls.push_back(b);
			}

			rjSender.send(visionPacket);
		}
	}
	
	return 0;
}
