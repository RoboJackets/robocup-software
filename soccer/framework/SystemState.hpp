#pragma once

#include <Robot.hpp>

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
class OurRobot;
class OpponentRobot;

namespace Packet
{
	class LogFrame;
}

struct Ball
{
	Ball()
	{
		valid = false;
	}
	
	Geometry2d::Point pos;
	Geometry2d::Point vel;
	Geometry2d::Point accel;
	bool valid;
};

class SystemState
{
	public:
		// Debug graphics
		void drawLine(const Geometry2d::Line &line, const QColor &color = Qt::black, const QString &layer = QString());
		
		void drawLine(const Geometry2d::Point &p0, const Geometry2d::Point &p1, const QColor &color = Qt::black, const QString &layer = QString())
		{
			drawLine(Geometry2d::Line(p0, p1), color);
		}
		
		void drawCircle(const Geometry2d::Point &center, float radius, const QColor &color = Qt::black, const QString &layer = QString());
		void drawPath(const Geometry2d::Point *pts, int n, const QColor &color = Qt::black, const QString &layer = QString());
		void drawPolygon(const Geometry2d::Point *pts, int n, const QColor &color = Qt::black, const QString &layer = QString());
		void drawPolygon(const std::vector<Geometry2d::Point>& pts, const QColor &color = Qt::black, const QString &layer = QString());
		void drawText(const QString &text, const Geometry2d::Point &pos, const QColor &color = Qt::black, const QString &layer = QString());
		void drawObstacle(const ObstaclePtr& obs, const QColor &color = Qt::black, const QString &layer = QString());
		void drawObstacles(const ObstacleGroup& group, const QColor &color = Qt::black, const QString &layer = QString());
		
		uint64_t timestamp;
		GameState gameState;
		
		// All possible robots.
		// Robots that aren't on the field are present here because a robot may be removed and replaced,
		// and that particular robot may be important (e.g. goalie).
		//
		// Plays need to keep Robot*'s around, so we can't just delete the robot since the play needs
		// to see that it is no longer visible.  We don't want multiple Robots for the same shell because
		// that would give the appearance that a new robot appeared when it was actually just pushed back on
		// the field.
		std::vector<OurRobot *> self;
		std::vector<OpponentRobot *> opp;
		
		Ball ball;
		boost::shared_ptr<Packet::LogFrame> logFrame;
		
		SystemState();
		~SystemState();
		
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
