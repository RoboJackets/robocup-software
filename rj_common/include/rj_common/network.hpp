#pragma once

#include <string>

// Here's how networking works:
//
// Vision:  Shared or simulated.
//    Shared vision is the system used in competition.  It sends multicast data
//    to 224.5.23.2 port 10002.
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

/*
 * These IP addresses are the multicast addresses we expect referee and vision
 * data to come from, respectively. These are given by the league.
 *
 * In networking terms, the referee packets' source address should match
 * kRefereeSourceAddress, and same thing for SharedVisionAddress.
 *
 * The physical interface we expect to receive these packets from is defined
 * below.
 */
static const std::string kRefereeSourceAddress = "224.5.23.1";
static const std::string kSharedVisionSourceAddress = "224.5.23.2";

/*
 * UPDATE (4/9/2023): these are no longer necessary. Both VisionReceiver and
 * ExternalReferee now simply join the multicast group and listen for their
 * respective source addresses above.
 *
 * These IP addresses are the interfaces (e.g. Ethernet plugged into this
 * laptop) where we expect ref/vision data to come from. They are (likely)
 * physical Ethernet links to the network, so this address won't show up in
 * Wireshark.
 *
 * When running sim, this should be localhost = 127.0.0.1.
 *
 * Run ifconfig to see list of interfaces on this computer, and pick the right
 * one (or try them all in worst-case).
 */

// static const std::string kRefereeInterface = "192.168.20.119";
static const std::string kRefereeInterface = "127.0.0.1";
static const std::string kVisionInterface =
    kRefereeInterface;  // In all but rare cirucmstances, this should match kRefereeInterface.

// The network address of the base station
static const std::string kBaseStationAddress = "10.42.0.248";
// The Port (on the local machine) to bind the control message socket to
static const int kControlMessageSocketPort = 8000;
// The Port (on the local machine) to bind the robot status socket to
static const int kRobotStatusMessageSocketPort = 8001;
// The Port (on the local machine) to bind the alive robots socket to
static const int kAliveRobotsMessageSocketPort = 8002;

static const int kSimVisionPort = 10020;  // was 10020 before 1-30-2022
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

// This param is now a ROS param loaded via launch file.
// static constexpr int kNetworkRadioServerPort = 25565;
static constexpr int kRobotEndpointPort = 25566;

// Double-sized field port with old Protobuf protocol
// static const int SharedVisionPortDoubleOld = 10005;

// Double-sized field port with new Protobuf protocol
static const int kSharedVisionPortDoubleNew = 10006;

static const int kRadioRxPort = 12000;
static const int kRadioTxPort = 13000;
