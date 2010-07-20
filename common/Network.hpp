#pragma once

// Here's how networking works:
//
// Vision:  Shared or simulated.
//    Shared vision is the system used in competition.  It sends multicast data to 224.5.20.2 port 10002.
//    Simulated vision is sent by our simulator to 127.0.0.1 ports 10000 and 10001.
//      The data sent to the two simulated vision ports is identical
//      It's sent to two ports so we can stay on localhost since multicast
//      doesn't work if there are no routing entries (a common case during development).
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
// The network ports are set in Processor's constructor and don't change after that.
// They are determined by command-line options (-sim and -r).
// If no radio channel is given on the command line, the first available one is picked
// based on which soccer-side port can be bound.

static const char SharedVisionAddress[] = "224.5.20.2";
static const char RefereeAddress[] = "224.5.23.1";

static const int SimCommandPort = 8999;
static const int SimVisionPort = 9000;
static const int RefereePort = 10001;
static const int SharedVisionPort = 10002;
static const int RadioRxPort = 12000;
static const int RadioTxPort = 13000;
