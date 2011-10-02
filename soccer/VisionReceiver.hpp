#pragma once

#include <protobuf/messages_robocup_ssl_wrapper.pb.h>

#include <QThread>
#include <QMutex>

#include <vector>

#include <stdint.h>

class QUdpSocket;

class VisionPacket
{
public:
	// Local time when the packet was received
	uint64_t receivedTime;
	
	// protobuf message from the vision system
	SSL_WrapperPacket wrapper;
};

class VisionReceiver: public QThread
{
public:
	VisionReceiver(bool sim);

	void stop();
	
	// Copies the vector of packets and then clears it.
	// The vector contains only packets received since the last time this was called
	// (or since the VisionReceiver was started, if getPackets has never been called).
	//
	// The caller is responsible for freeing the packets after this function returns.
	void getPackets(std::vector<VisionPacket *> &packets);
	
protected:
	virtual void run();
	
	bool _simulation;
	volatile bool _running;
	
	// This mutex protects the vector of packets
	QMutex _mutex;
	std::vector<VisionPacket *> _packets;
};
