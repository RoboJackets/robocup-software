#pragma once

#include <string>

/*
 * These IP addresses are the multicast addresses we expect referee and vision
 * data to come from, respectively. These are given by the league.
 *
 * In networking terms, the referee packets' source address should match
 * kRefereeSourceAddress, and same thing for SharedVisionAddress.
 */
static const std::string kRefereeSourceAddress = "224.5.23.1";
static const std::string kSharedVisionSourceAddress = "224.5.23.2";

// The network address of the base station
static const std::string kBaseStationAddress = "10.42.0.248";
// The Port (on the local machine) to bind the control message socket to
static const int kControlMessageSocketPort = 8000;
// The Port (on the local machine) to bind the robot status socket to
static const int kRobotStatusMessageSocketPort = 8001;
// The Port (on the local machine) to bind the alive robots socket to
static const int kAliveRobotsMessageSocketPort = 8002;

static const int kSimBlueStatusPort = 30011;
static const int kSimYellowStatusPort = 30012;
static const int kSimCommandPort = 10300;
static const int kSimBlueCommandPort = 10301;
static const int kSimYellowCommandPort = 10302;

static const int kProtobufRefereePort = 10003;
