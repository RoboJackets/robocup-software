#pragma once

#include <QVector>
#include <QVector3D>
#include <QMap>
#include <QTimer>
#include <QMutex>
#include <QUdpSocket>
#include <QDomElement>
#include <QString>
#include <sys/time.h>

#include <Geometry2d/Point.hpp>

#include <protobuf/SimCommand.pb.h>

#include "Ball.hpp"
#include "Robot.hpp"
#include "Field.hpp"

class SSL_DetectionRobot;

class Environment : public QObject
{
	Q_OBJECT;

public:
	typedef QMap<unsigned int, Robot*> RobotMap;

private:
	// IF true, the next vision frame is dropped.
	// Automatically cleared.
	bool _dropFrame;

	Field* _field;

	RobotMap _blue;
	RobotMap _yellow;
	QVector<Ball*> _balls;

	QString _configFile; //< filename for the config file

	// This timer causes physics to be stepped on a regular basis
	QTimer _timer;

	QUdpSocket _visionSocket;   ///< Simulated vision - can also receive commands from soccer
	QUdpSocket _radioSocketBlue, _radioSocketYellow; ///< Connections for robots

	struct timeval _lastStepTime;

	// How many vision frames we've sent
	int _frameNumber;

	// How many physics steps have run since the last vision packet was sent
	int _stepCount;

public:
	// If true, send data to the shared vision multicast address.
	// If false, send data to the two simulated vision addresses.
	volatile bool sendShared;

	int ballVisibility;

	Environment(const QString& configFile, bool sendShared_);

	/** initializes the timer, connects sockets */
	void connectSockets();

	void dropFrame()
	{
		_dropFrame = true;
	}

	const QVector<Ball*> &balls() const {	return _balls; }

	const RobotMap &blue() const { return _blue; }
	const RobotMap &yellow() const { return _yellow; }

	/** add a ball @ pos */
	void addBall(Geometry2d::Point pos);

	/** add a robot with id i to the environment @ pos */
	void addRobot(bool blue, int id, const Geometry2d::Point& pos, Robot::RobotRevision rev);

	/** removes a robot with id i from the environment */
	void removeRobot(bool blue, int id);

//signals:
//	// connect to visualization for rendering
//	void setRobotPose(bool blue, int id, const QVector3D& pos, qreal angle, const QVector3D& axis);
//	void addNewRobot(bool blue, int id);
//	void removeExistingRobot(bool blue, int id);

public:

	/**
	 * Primary simulation step function - called by a timer at a fixed interval
	 */
	void step();

private:
	Robot *robot(bool blue, int board_id) const;
	static void convert_robot(const Robot *robot, SSL_DetectionRobot *out);
	void handleRadioTx(bool blue, const Packet::RadioTx& data);

	void handleSimCommand(const Packet::SimCommand& cmd);

	void sendVision();

	// Packet handling
	template<class PACKET>
	bool loadPacket(QUdpSocket& socket, PACKET& packet) {
		std::string buf;
		unsigned int n = socket.pendingDatagramSize();
		buf.resize(n);
		socket.readDatagram(&buf[0], n);

		if (!packet.ParseFromString(buf))
		{
			printf("Bad packet of %d bytes\n", n);
			return false;
		}
		return true;
	}

	// Returns true if any robot occludes a ball from a camera's point of view.
	bool occluded(Geometry2d::Point ball, Geometry2d::Point camera);

	// Config file handling
	bool loadConfigFile(const QString& filename);
	void procTeam(QDomElement e, bool blue);
};
