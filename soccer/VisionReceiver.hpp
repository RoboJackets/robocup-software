#pragma once

#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <Network.hpp>
#include <Utils.hpp>

#include <QThread>
#include <QMutex>
#include <vector>
#include <stdint.h>

class QUdpSocket;

class VisionPacket {
public:
    /// Local time when the packet was received
    Time receivedTime;

    /// protobuf message from the vision system
    SSL_WrapperPacket wrapper;
};

/**
 * @brief Receives vision packets over UDP and places them in a buffer until
 * they are read.
 *
 * @details When start() is called, a new thread is spawned that listens on a
 * UDP port for packets. If sim = true, it tries both simulator ports until one
 * works. Otherwise, it connects to the port specified in the constructor.
 *
 * Whenever a new packet comes in (encoded as Google Protobuf), it is parsed
 * into an SSL_WrapperPacket and placed onto the circular buffer @_packets.
 * They remain there until they are retrieved with getPackets().
 */
class VisionReceiver : public QThread {
public:
    VisionReceiver(bool sim = false, int port = SharedVisionPortDoubleOld);

    void stop();

    /// Copies the vector of packets and then clears it. The vector contains
    /// only packets received since the last time this was called (or since the
    /// VisionReceiver was started, if getPackets has never been called).
    ///
    /// The caller is responsible for freeing the packets after this function
    /// returns.
    void getPackets(std::vector<VisionPacket*>& packets);

    bool simulation;
    int port;

protected:
    virtual void run() override;

    volatile bool _running;

    /// This mutex protects the vector of packets
    QMutex _mutex;
    std::vector<VisionPacket*> _packets;
};
