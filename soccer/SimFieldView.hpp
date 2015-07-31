// An extension of FieldView that generates SimCommands in response
// to clicks/drags when live.

#pragma once

#include <FieldView.hpp>
#include <QUdpSocket>
#include <protobuf/SimCommand.pb.h>

class SimFieldView: public FieldView
{
	Q_OBJECT;
	
	public:
		SimFieldView(QWidget *parent = nullptr);
		
		void sendSimCommand(const Packet::SimCommand &cmd);
		
	Q_SIGNALS:
		// Emitted when the user selects a robot.
		// The robot is identified by shell number.
		// shell may be -1 to select no robot.
		void robotSelected(int shell);
	
	protected:
		virtual void mouseReleaseEvent(QMouseEvent*);
		virtual void mousePressEvent(QMouseEvent*);
		virtual void mouseMoveEvent(QMouseEvent*);
		
		virtual void drawTeamSpace(QPainter &p);
		
	private:
		// Places the ball at a position on the screen
		void placeBall(QPointF pos);
		
		QUdpSocket _simCommandSocket;
		
		// True while a line is being dragged from the ball
		enum
		{
			DRAG_NONE = 0,
			DRAG_PLACE,
			DRAG_SHOOT
		} _dragMode;
		
		int _dragRobot;
		int _dragRobotBlue;
		Geometry2d::Point _dragTo;
		Geometry2d::Point _shot;
};
