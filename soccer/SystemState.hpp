#pragma once

#include <vector>
#include <string>
#include <memory>

#include <QMap>
#include <QColor>

#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <GameState.hpp>
#include <Constants.hpp>
#include <Utils.hpp>
#include <Geometry2d/Arc.hpp>

class RobotConfig;
class OurRobot;
class OpponentRobot;

namespace Packet
{
	class LogFrame;
}


/**
 * @brief Our beliefs about the ball's position and velocity
 */
class Ball
{
public:
	Ball()
	{
		valid = false;
		time = 0;
	}

	Geometry2d::Point pos;
	Geometry2d::Point vel;
	bool valid;

	/// Time at which this estimate is valid
	Time time;
};

/**
 * @brief Holds the positions of everything on the field
 * @details  this has the debugging drawer for the gui
 * but it also contains the game state, so this is passed game state information
 * contains essentially everything data wise
 * used in all threads, this is the class that is passed to for data
 */
class SystemState
{
public:
	SystemState();
	~SystemState();


	/**
	 * @defgroup drawing_functions Drawing Functions
	 * These drawing functions add certain shapes/lines to the current LogFrame.
	 * Each time the FieldView updates, it reads the LogFrame and draws these items.
	 * This way debug data can be drawn on-screen and also logged.
	 *
	 * Each drawing function also associates the drawn content with a particular
	 * 'layer'.  Separating drawing items into layers lets you choose at runtime
	 * which items actually get drawn.
	 */

	/** @ingroup drawing_functions */
	void drawLine(const Geometry2d::Line &line, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawLine(const Geometry2d::Point &p0, const Geometry2d::Point &p1, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawCircle(const Geometry2d::Point &center, float radius, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawArc(const Geometry2d::Arc &arc, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
    void drawPolygon(const Geometry2d::Polygon &pts, const QColor &color = Qt::black, const QString &layer = QString());
    /** @ingroup drawing_functions */
	void drawPolygon(const Geometry2d::Point *pts, int n, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawPolygon(const std::vector<Geometry2d::Point>& pts, const QColor &color = Qt::black, const QString &layer = QString());
    /** @ingroup drawing_functions */
	void drawText(const QString &text, const Geometry2d::Point &pos, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawShape(const std::shared_ptr<Geometry2d::Shape>& obs, const QColor &color = Qt::black, const QString &layer = QString());
	/** @ingroup drawing_functions */
	void drawCompositeShape(const Geometry2d::CompositeShape& group, const QColor &color = Qt::black, const QString &layer = QString());

	Time timestamp;
	GameState gameState;

	/// All possible robots.
	/// Robots that aren't on the field are present here because a robot may be removed and replaced,
	/// and that particular robot may be important (e.g. goalie).
	///
	/// Plays need to keep Robot*'s around, so we can't just delete the robot since the play needs
	/// to see that it is no longer visible.  We don't want multiple Robots for the same shell because
	/// that would give the appearance that a new robot appeared when it was actually just pushed back on
	/// the field.
	std::vector<OurRobot *> self;
	std::vector<OpponentRobot *> opp;

	Ball ball;
	std::shared_ptr<Packet::LogFrame> logFrame;

	const QStringList &debugLayers() const
	{
		return _debugLayers;
	}

	/// Returns the number of a debug layer given its name
	int findDebugLayer(QString layer);

private:

	/// Map from debug layer name to ID
	QMap<QString, int> _debugLayerMap;

	/// Debug layers in order by ID
	QStringList _debugLayers;

	/// Number of debug layers
	int _numDebugLayers;
};
