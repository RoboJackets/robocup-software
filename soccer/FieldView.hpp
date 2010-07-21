// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QWidget>
#include <QUdpSocket>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <protobuf/LogFrame.pb.h>
#include <protobuf/SimCommand.pb.h>

class Logger;

/** class that performs drawing of log data onto the field */
class FieldView : public QWidget
{
	Q_OBJECT;

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
		
		Logger *logger;
		
		bool showRawRobots;
		bool showRawBalls;
		bool showCoords;
		
		// Returns the frame for _frameNumber.
		// This is used by other GUI code to avoid reading the frame repeatedly.
		const Packet::LogFrame &frame() const
		{
			return _frame;
		}
		
		// Sets the current frame number and reads the frame
		void frameNumber(int n);
		
		void rotate(int value);
		
		void sendSimCommand(const Packet::SimCommand &cmd);
	
	Q_SIGNALS:
		// Emitted when the user selects a robot.
		// The robot is identified by shell number.
		// shell may be -1 to select no robot.
		void robotSelected(int shell);
	
	protected:
		virtual void paintEvent(QPaintEvent* pe);
		virtual void resizeEvent(QResizeEvent *e);
		
		virtual void mouseReleaseEvent(QMouseEvent*);
		virtual void mousePressEvent(QMouseEvent*);
		virtual void mouseMoveEvent(QMouseEvent*);
		virtual void mouseDoubleClickEvent(QMouseEvent*);
		
		// Places the ball at a position on the screen
		void placeBall(QPointF pos);
		
		void drawText(QPainter& p, QPointF pos, QString text, bool center = true);
		void drawField(QPainter&);
		void drawRobot(QPainter& p, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall = false);
		void drawCoords(QPainter& p);

	protected:
		QUdpSocket _simCommandSocket;

		// Coordinate transformations
		Geometry2d::TransformMatrix _screenToWorld;
		Geometry2d::TransformMatrix _worldToTeam;
		Geometry2d::TransformMatrix _teamToWorld;
		
		// Rotation of the field in 90-degree increments (0 to 3).
		int _rotate;
		
		// How many degrees to rotate text so it shows up the right way on screen
		int _textRotation;
		
		// True while a line is being dragged from the ball
		enum
		{
			DRAG_NONE = 0,
			DRAG_PLACE,
			DRAG_SHOOT
		} _dragMode;
		
		Geometry2d::Point _dragTo;
		Geometry2d::Point _shot;
		
		// Most recent frame
		Packet::LogFrame _frame;
		
		// Sequence number of the LogFrame in _frame.
		// Since frames can be added while we're drawing history,
		// this is needed to look back from a single point in time.
		int _frameNumber;
		
		QVector<bool> _layerVisible;
};
