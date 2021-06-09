#pragma once

#include <string>

// Here's how networking works:
//
// Vision:  Shared or simulated.
//    Shared vision is the system used in competition.  It sends multicast data
//    to 224.5.20.2 port 10002.
//    Simulated vision is sent by our simulator to 127.0.0.1 ports 10000 and
//    10001.
//      The data sent to the two simulated vision ports is identical
//      It's sent to two ports so we can stay on localhost since multicast
//      doesn't work if there are no routing entries (a common case during
//      development).
//
// Radio:
//    There are two radio channels from the network's point of view.
//    Currently the hardware only supports one, but this needs to be fixed.
//    Radio packets stay on localhost.
//
//    Soccer uses ports 12000 and 12001.
//    Radio uses ports 13000 and 13001.
//
//    RadioTx packets go from soccer to radio.
//    RadioRx packets go from radio to soccer.
//
// The network ports are set in Processor's constructor and don't change after
// that. They are determined by command-line options (-sim and -r). If no radio
// channel is given on the command line, the first available one is picked based
// on which soccer-side port can be bound.

static const std::string kRefereeAddress = "224.5.23.1";
static const std::string kSharedVisionAddress = "224.5.23.2";

static const int kSimVisionPort = 10020;
static const int kSimBlueStatusPort = 30011;
static const int kSimYellowStatusPort = 30012;
static const int kSimCommandPort = 10300;
static const int kSimBlueCommandPort = 10301;
static const int kSimYellowCommandPort = 10302;

static const int kLegacyRefereePort = 10001;
static const int kProtobufRefereePort = 10003;

// Kept around for legacy code
static const int kSharedVisionPort = 10002;

// Primary Single-sized field port with old Protobuf protocol
static const int kSharedVisionPortSinglePrimary = 10002;

// Secondary Single-sized field port with old Protobuf protocol
static const int kSharedVisionPortSingleSecondary = 10004;

static constexpr int kNetworkRadioServerPort = 25565;

// Double-sized field port with old Protobuf protocol
// static const int SharedVisionPortDoubleOld = 10005;

// Double-sized field port with new Protobuf protocol
static const int kSharedVisionPortDoubleNew = 10006;

static const int kRadioRxPort = 12000;
static const int kRadioTxPort = 13000;
