// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QGLWidget>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <protobuf/LogFrame.pb.h>

#include <set>
#include <boost/shared_ptr.hpp>

class Logger;

/** class that performs drawing of log data onto the field */
class FieldView : public QWidget
{
	public:
		FieldView(QWidget* parent = 0);
		
		void layerVisible(int i, bool value)
		{
			if (i >= 0 && i < _layerVisible.size())
			{
				_layerVisible[i] = value;
			}
		}
		
		bool layerVisible(int i) const
		{
			if (i < _layerVisible.size())
			{
				return true;
			} else {
				return false;
			}
		}
		
		void history(const std::vector<boost::shared_ptr<Packet::LogFrame> > *value)
		{
			_history = value;
		}
		
		void rotate(int value);
		
		// True if this control is showing live (vs. historical) data.
		// If false, it will draw a red border.
		bool live;
		
		bool showRawRobots;
		bool showRawBalls;
		bool showCoords;
		
		// Which robots will show a command trace
		std::set<int> showCommandTrace;
		
	protected:
		virtual void mouseDoubleClickEvent(QMouseEvent* e);
		virtual void paintEvent(QPaintEvent* e);
		virtual void resizeEvent(QResizeEvent *e);
		
		virtual void drawWorldSpace(QPainter &p);
		virtual void drawTeamSpace(QPainter &p);
		
		void drawText(QPainter& p, QPointF pos, QString text, bool center = true);
		void drawField(QPainter& p, const Packet::LogFrame *frame);
		void drawRobot(QPainter& p, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall = false, bool faulty = false);
		void drawCoords(QPainter& p);

	protected:
		// Returns a pointer to the most recent frame, or null if none is available.
		boost::shared_ptr<Packet::LogFrame> currentFrame();
		
		// Coordinate transformations
		Geometry2d::TransformMatrix _screenToWorld;
		Geometry2d::TransformMatrix _worldToTeam;
		Geometry2d::TransformMatrix _teamToWorld;
		
		// Rotation of the field in 90-degree increments (0 to 3).
		int _rotate;
		
		// How many degrees to rotate text so it shows up the right way on screen
		int _textRotation;
		
		const std::vector<boost::shared_ptr<Packet::LogFrame> > *_history;
		
		QVector<bool> _layerVisible;
};
