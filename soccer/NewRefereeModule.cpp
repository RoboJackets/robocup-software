#include "NewRefereeModule.hpp"

#include <Network.hpp>
#include <multicast.hpp>
#include <Utils.hpp>
#include <unistd.h>
#include <QMutexLocker>
#include <QUdpSocket>
#include <stdexcept>

namespace NewRefereeModuleEnums
{
std::string stringFromStage(Stage s)
{
	switch(s)
	{
	case NORMAL_FIRST_HALF_PRE: return "Normal First Half Prep";
	case NORMAL_FIRST_HALF: return "Normal First Half";
	case NORMAL_HALF_TIME: return "Normal Half Time";
	case NORMAL_SECOND_HALF_PRE: return "Normal Second Half Prep";
	case NORMAL_SECOND_HALF: return "Normal Second Half";
	case EXTRA_TIME_BREAK: return "Extra Time Break";
	case EXTRA_FIRST_HALF_PRE: return "Extra First Half Prep";
	case EXTRA_FIRST_HALF: return "Extra First Half";
	case EXTRA_HALF_TIME: return "Extra Half Time";
	case EXTRA_SECOND_HALF_PRE: return "Extra Second Half Prep";
	case EXTRA_SECOND_HALF: return "Extra Second Half";
	case PENALTY_SHOOTOUT_BREAK: return "Penalty Shootout Break";
	case PENALTY_SHOOTOUT: return "Penalty Shootout";
	case POST_GAME: return "Post Game";
	default: return "";
	}
}

std::string stringFromCommand(Command c)
{
	switch(c)
	{
	case HALT: return "Halt";
	case STOP: return "Stop";
	case NORMAL_START: return "Normal Start";
	case FORCE_START: return "Force Start";
	case PREPARE_KICKOFF_YELLOW: return "Yellow Kickoff Prep";
	case PREPARE_KICKOFF_BLUE: return "Blue Kickoff Prep";
	case PREPARE_PENALTY_YELLOW: return "Yellow Penalty Prep";
	case PREPARE_PENALTY_BLUE: return "Blue Penalty Prep";
	case DIRECT_FREE_YELLOW: return "Direct Yellow Free Kick";
	case DIRECT_FREE_BLUE: return "Direct Blue Free Kick";
	case INDIRECT_FREE_YELLOW: return "Indirect Yellow Free Kick";
	case INDIRECT_FREE_BLUE: return "Indirect Blue Free Kick";
	case TIMEOUT_YELLOW: return "Timeout Yellow";
	case TIMEOUT_BLUE: return "Timeout Blue";
	case GOAL_YELLOW: return "Goal Yellow";
	case GOAL_BLUE: return "Goal Blue";
	default: return "";
	}
}
}

using namespace std;
using namespace NewRefereeModuleEnums;

NewRefereeModule::NewRefereeModule()
	: stage(NORMAL_FIRST_HALF_PRE),
	  command(HALT),
	  _running(false)
{
}

NewRefereeModule::~NewRefereeModule()
{
	this->stop();
}

void NewRefereeModule::stop()
{
	_running = false;
	wait();
}

void NewRefereeModule::getPackets(std::vector<NewRefereePacket *> &packets)
{
	_mutex.lock();
	packets = _packets;
	_packets.clear();
	_mutex.unlock();
}

void NewRefereeModule::run()
{
	QUdpSocket socket;

	if(!socket.bind(ProtobufRefereePort, QUdpSocket::ShareAddress))
	{
		throw runtime_error("Can't bind to shared referee port");
	}

	multicast_add(&socket, RefereeAddress);

	_packets.reserve(4);

	_running = true;
	while(_running)
	{
		char buf[65536];

		if(!socket.waitForReadyRead(500))
		{
			continue;
		}

		QHostAddress host;
		quint16 port = 0;
		qint64 size = socket.readDatagram(buf, sizeof(buf), &host, &port);
		if(size < 1)
		{
			fprintf(stderr, "NewRefereeModule: %s/n", (const char *)socket.errorString().toAscii());
			::usleep(100000);
			continue;
		}

		NewRefereePacket *packet = new NewRefereePacket;
		packet->receivedTime = timestamp();
		if(!packet->wrapper.ParseFromArray(buf, size))
		{
			fprintf(stderr, "NewRefereeModule: got bad packet of %d bytes from %s:%d\n", (int)size, (const char *)host.toString().toAscii(), port);
			fprintf(stderr, "Packet: %s\n", buf);
			fprintf(stderr, "Address: %s\n", RefereeAddress);
			continue;
		}

		_mutex.lock();
		_packets.push_back(packet);

		stage = (Stage)packet->wrapper.stage();
		command = (Command)packet->wrapper.command();
		sent_time = packet->wrapper.packet_timestamp();
		stage_time_left = packet->wrapper.stage_time_left();
		command_counter = packet->wrapper.command_counter();
		command_timestamp = packet->wrapper.command_timestamp();
		yellow_info.ParseRefboxPacket(packet->wrapper.yellow());
		blue_info.ParseRefboxPacket(packet->wrapper.blue());
		
		_mutex.unlock();

	}
}