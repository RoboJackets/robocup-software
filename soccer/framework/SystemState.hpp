#pragma once

#include <vector>
#include <string>

#include <QMap>
#include <QColor>

#include <Geometry2d/Point.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <GameState.hpp>
#include <framework/Obstacle.hpp>
#include <framework/MotionCmd.hpp>
#include <Constants.hpp>

class RobotConfig;

namespace Packet
{
	class LogFrame;
}

class SystemState
{
	public:
		class Robot
		{
			public:
				boost::shared_ptr<RobotConfig> config;
				
				ObstacleGroup obstacles;
				
				uint8_t shell;
				enum Rev
				{
					rev2008 = 0,
					rev2010 = 1
				};
				
				Rev rev;
				Geometry2d::Point pos;
				Geometry2d::Point vel;
				float angle;
				float angleVel;
				MotionCmd cmd;
				bool valid;
				bool hasBall;
				Geometry2d::Point cmd_vel;
				float cmd_w;
				Packet::RadioTx::Robot *radioTx;
				Packet::RadioRx radioRx;
				
				Robot()
				{
					shell = 0;
					rev = rev2008;
					angle = 0;
					angleVel = 0;
					valid = false;
					hasBall = false;
					cmd_w = 0;
					radioTx = 0;
				}
		};
		
		class Ball
		{
			public:
				Geometry2d::Point pos;
				Geometry2d::Point vel;
				Geometry2d::Point accel;
				bool valid;
				
				Ball()
				{
					valid = false;
				}
		};
		
		enum Possession
		{
			OFFENSE = 0,
			DEFENSE = 1,
			FREEBALL = 2
		};
		
		enum BallFieldPos
		{
			HOMEFIELD = 0,
			MIDFIELD = 1,
			OPPFIELD = 2
		};
		
		class GameStateID
		{
			public:
				Possession posession;
				BallFieldPos field_pos;
				
				GameStateID()
				{
					posession = OFFENSE;
					field_pos = HOMEFIELD;
				}
		};
		
		// Debug graphics
		void drawLine(const Geometry2d::Line &line, const QColor &color = Qt::black, const QString &layer = QString());
		
		void drawLine(const Geometry2d::Point &p0, const Geometry2d::Point &p1, const QColor &color = Qt::black, const QString &layer = QString())
		{
			drawLine(Geometry2d::Line(p0, p1), color);
		}
		
		void drawCircle(const Geometry2d::Point &center, float radius, const QColor &color = Qt::black, const QString &layer = QString());
		void drawPath(const Geometry2d::Point *pts, int n, const QColor &color = Qt::black, const QString &layer = QString());
		void drawPolygon(const Geometry2d::Point *pts, int n, const QColor &color = Qt::black, const QString &layer = QString());
		void drawText(const QString &text, const Geometry2d::Point &pos, const QColor &color = Qt::black, const QString &layer = QString());
		
		GameStateID stateID;
		uint64_t timestamp;
		GameState gameState;
		Robot self[Constants::Robots_Per_Team];
		Robot opp[Constants::Robots_Per_Team];
		Ball ball;
		boost::shared_ptr<Packet::LogFrame> logFrame;
		
		SystemState()
		{
			timestamp = 0;
			_numDebugLayers = 0;
		}
		
		const QStringList &debugLayers() const
		{
			return _debugLayers;
		}
		
	private:
		// Returns the number of a debug layer given its name
		int findDebugLayer(QString layer);
		
		// Map from debug layer name to ID
		typedef QMap<QString, int> DebugLayerMap;
		DebugLayerMap _debugLayerMap;
		
		// Debug layers in order by ID
		QStringList _debugLayers;
		
		// Number of debug layers
		int _numDebugLayers;
};
