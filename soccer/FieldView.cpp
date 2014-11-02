
#include <FieldView.hpp>

#include <stdio.h>

#include <Network.hpp>
#include <LogUtils.hpp>
#include <Constants.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>

#include <QStyleOption>
#include <QLayout>
#include <QPainter>
#include <QResizeEvent>
#include <boost/foreach.hpp>
#include <algorithm>
#include <sys/socket.h>

#include <VisionDotPattern.hpp>

using namespace std;
using namespace boost;
using namespace Packet;

QColor ballColor(0xff, 0x90, 0);

FieldView::FieldView(QWidget* parent) :
	QWidget(parent)
{
	showRawRobots = false;
	showRawBalls = false;
	showCoords = false;
    showDotPatterns = false;
	_rotate = 1;
	_history = 0;

	// Green background
	QPalette p = palette();
	p.setColor(QPalette::Window, QColor(0, 85.0, 0));
	setPalette(p);
	setAutoFillBackground(true);
}

std::shared_ptr<LogFrame> FieldView::currentFrame()
{
	if (_history && !_history->empty())
	{
		return _history->at(0);
	} else {
		return std::shared_ptr<LogFrame>();
	}
}

void FieldView::rotate(int value)
{
	_rotate = value;
	
	// Fix size
	updateGeometry();
	
	update();
}

void FieldView::paintEvent(QPaintEvent* e)
{
	QPainter p(this);
	
	if (!live)
	{
		// Non-live border
		p.setPen(QPen(Qt::red, 4));
		p.drawRect(rect());
	}
	
	// Set up world space
	p.translate(width() / 2.0, height() / 2.0);
	p.scale(width(), -height());
	p.rotate(_rotate * 90);
	p.scale(1.0 / Floor_Length, 1.0 / Floor_Width);
	
	// Set text rotation for world space
	_textRotation = -_rotate * 90;
	
	if (showCoords)
	{
		drawCoords(p);
	}
	
	// Get the latest LogFrame
	const std::shared_ptr<LogFrame> frame = currentFrame();
	
	if (!frame)
	{
		// No data available yet
		return;
	}
	
	// Check number of debug layers
	if (_layerVisible.size() != frame->debug_layers_size())
	{
		int start = _layerVisible.size();
		_layerVisible.resize(frame->debug_layers_size());
		
		// Turn on the new layers
		for (int i = start; i < _layerVisible.size(); ++i)
		{
			_layerVisible[i] = true;
		}
	}
	
	// Make coordinate transformations
	_screenToWorld = Geometry2d::TransformMatrix();
	_screenToWorld *= Geometry2d::TransformMatrix::scale(Floor_Length, Floor_Width);
	_screenToWorld *= Geometry2d::TransformMatrix::rotate(-_rotate * M_PI / 2.0);
	_screenToWorld *= Geometry2d::TransformMatrix::scale(1.0 / width(), -1.0 / height());
	_screenToWorld *= Geometry2d::TransformMatrix::translate(-width() / 2.0, -height() / 2.0);
	
	_worldToTeam = Geometry2d::TransformMatrix();
	_worldToTeam *= Geometry2d::TransformMatrix::translate(0, Field_Length / 2.0f);
	if (frame->defend_plus_x())
	{
		_worldToTeam *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
	} else {
		_worldToTeam *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
	}
	
	_teamToWorld = Geometry2d::TransformMatrix();
	if (frame->defend_plus_x())
	{
		_teamToWorld *= Geometry2d::TransformMatrix::rotate(M_PI / 2.0);
	} else {
		_teamToWorld *= Geometry2d::TransformMatrix::rotate(-M_PI / 2.0);
	}
	_teamToWorld *= Geometry2d::TransformMatrix::translate(0, -Field_Length / 2.0f);
	
	// Draw world-space graphics
	drawWorldSpace(p);
	
	// Everything after this point is drawn in team space.
	// Transform that into world space depending on defending goal.
	if (frame->defend_plus_x())
	{
		p.rotate(90);
	} else {
		p.rotate(-90);
	}
	p.translate(0, -Field_Length / 2.0f);

	// Text has to be rotated so it is always upright on screen
	_textRotation = -_rotate * 90 + (frame->defend_plus_x() ? -90 : 90);
	
	drawTeamSpace(p);
}

void FieldView::drawWorldSpace(QPainter& p)
{
	// Get the latest LogFrame
	const LogFrame *frame = _history->at(0).get();
	
	// Draw the field
	drawField(p, frame);
	


	///	draw a comet trail behind each robot so we can see its path easier
	int pastLocationCount = 50;
	const float prev_loc_scale = 0.4;
	for (int i = 1; i < pastLocationCount + 1 && i < _history->size(); i++) {
		const LogFrame *oldFrame = _history->at(i).get();
		if (oldFrame) {
			for (const SSL_WrapperPacket &wrapper : oldFrame->raw_vision()) {
				if (!wrapper.has_detection()) {
					//	useless
					continue;
				}

				const SSL_DetectionFrame &detect = wrapper.detection();

				float alpha = 0.6f * (1.0f - (float)i / pastLocationCount);

				QColor blue = Qt::blue;
				blue.setAlphaF(alpha);
				p.setPen(blue);
				p.setBrush(QBrush(blue));
				for (const SSL_DetectionRobot &r : detect.robots_blue()) {
					QPointF pos(r.x() / 1000, r.y() / 1000);
					p.drawEllipse(pos, Robot_Radius * prev_loc_scale, Robot_Radius * prev_loc_scale);
				}

				QColor yellow = Qt::yellow;
				yellow.setAlphaF(alpha);
				p.setBrush(QBrush(yellow));
				p.setPen(yellow);
				for (const SSL_DetectionRobot &r : detect.robots_yellow()) {
					QPointF pos(r.x() / 1000, r.y() / 1000);
					p.drawEllipse(pos, Robot_Radius * prev_loc_scale, Robot_Radius * prev_loc_scale);
				}
			}
		}
	}


	// Raw vision
	if (showRawBalls || showRawRobots)
	{
		p.setPen(QColor(0xcc, 0xcc, 0xcc));
		BOOST_FOREACH(const SSL_WrapperPacket& wrapper, frame->raw_vision())
		{
			if (!wrapper.has_detection())
			{
				// Useless
				continue;
			}
			
			const SSL_DetectionFrame &detect = wrapper.detection();
			
			if (showRawRobots)
			{
				BOOST_FOREACH(const SSL_DetectionRobot& r, detect.robots_blue())
				{
					QPointF pos(r.x() / 1000, r.y() / 1000);
					drawRobot(p, true, r.robot_id(), pos, r.orientation());
// 					p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Robot_Radius, Robot_Radius);
				}
				
				BOOST_FOREACH(const SSL_DetectionRobot& r, detect.robots_yellow())
				{
					QPointF pos(r.x() / 1000, r.y() / 1000);
					drawRobot(p, false, r.robot_id(), pos, r.orientation());
// 					p.drawEllipse(QPointF(r.x() / 1000, r.y() / 1000), Robot_Radius, Robot_Radius);
				}
			}
			
			if (showRawBalls)
			{
				BOOST_FOREACH(const SSL_DetectionBall& b, detect.balls())
				{
					p.drawEllipse(QPointF(b.x() / 1000, b.y() / 1000), Ball_Radius, Ball_Radius);
				}
			}
		}
	}
}

void FieldView::drawTeamSpace(QPainter& p)
{
	// Get the latest LogFrame
	const LogFrame *frame = _history->at(0).get();
	
	// Block off half the field
	if (!frame->use_our_half())
	{
		const float FX = Floor_Width / 2;
		const float FY1 = -Field_Border;
		const float FY2 = Field_Length / 2;
		p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)), QColor(0, 0, 0, 128));
	}
	if (!frame->use_opponent_half())
	{
		const float FX = Floor_Width / 2;
		const float FY1 = Field_Length / 2;
		const float FY2 = Field_Length + Field_Border;
		p.fillRect(QRectF(QPointF(-FX, FY1), QPointF(FX, FY2)), QColor(0, 0, 0, 128));
	}
	
	if (showCoords)
	{
		drawCoords(p);
	}
	
	// History
	p.setBrush(Qt::NoBrush);
	for (unsigned int i = 1; i < 200 && i < _history->size(); ++i)
	{
		const LogFrame *oldFrame = _history->at(i).get();
		if (oldFrame && oldFrame->has_ball())
		{
			QPointF pos = qpointf(oldFrame->ball().pos());
			
			QColor c = ballColor;
			c.setAlpha(255 - i);
			p.setPen(c);
			
			p.drawEllipse(QRectF(-Ball_Radius + pos.x(), -Ball_Radius + pos.y(),
					Ball_Diameter, Ball_Diameter));
		}
	}
	
	// Debug lines
	BOOST_FOREACH(const DebugPath& path, frame->debug_paths())
	{
		if (path.layer() < 0 || layerVisible(path.layer()))
		{
			p.setPen(qcolor(path.color()));
			std::vector<QPointF> pts;
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts.push_back( qpointf(path.points(i)) );
			}
			p.drawPolyline(pts.data(), pts.size());
		}
	}

	// Debug circles
	BOOST_FOREACH(const DebugCircle& c, frame->debug_circles())
	{
		if (c.layer() < 0 || layerVisible(c.layer()))
		{
			p.setPen(qcolor(c.color()));
			p.drawEllipse(qpointf(c.center()), c.radius(), c.radius());
		}
	}

	// Debug text
	BOOST_FOREACH(const DebugText& text, frame->debug_texts())
	{
		if (text.layer() < 0 || layerVisible(text.layer()))
		{
			p.setPen(qcolor(text.color()));
			drawText(p, qpointf(text.pos()), QString::fromStdString(text.text()), text.center());
		}
	}

	// Debug polygons
	p.setPen(Qt::NoPen);
	BOOST_FOREACH(const DebugPath& path, frame->debug_polygons())
	{
		if (path.layer() < 0 || layerVisible(path.layer()))
		{
			if (path.points_size() < 3)
			{
				fprintf(stderr, "Ignoring DebugPolygon with %d points\n", path.points_size());
				continue;
			}
			
			QColor color = qcolor(path.color());
			color.setAlpha(64);
			p.setBrush(color);
			std::vector<QPointF> pts;
			for (int i = 0; i < path.points_size(); ++i)
			{
				pts.push_back( qpointf(path.points(i)) );
			}
			p.drawConvexPolygon(pts.data(), pts.size());
		}
	}
	p.setBrush(Qt::NoBrush);

	// Text positioning vectors
	QPointF rtX = qpointf(Geometry2d::Point(0, 1).rotated(-_rotate * 90));
	QPointF rtY = qpointf(Geometry2d::Point(-1, 0).rotated(-_rotate * 90));
	
	// Opponent robots
	BOOST_FOREACH(const LogFrame::Robot &r, frame->opp())
	{
		drawRobot(p, !frame->blue_team(), r.shell(), qpointf(r.pos()), r.angle(), r.ball_sense_status() == HasBall);
	}
	
	// Our robots
	int manualID = frame->manual_id();
	BOOST_FOREACH(const LogFrame::Robot &r, frame->self())
	{
		QPointF center = qpointf(r.pos());
		
		bool faulty = false;
		if (r.has_ball_sense_status() && (r.ball_sense_status() == Dazzled || r.ball_sense_status() == Failed))
		{
			faulty = true;
		}
		if (r.has_kicker_works() && !r.kicker_works())
		{
// 			faulty = true;
		}
		for (int i = 0; i < r.motor_status().size(); ++i)
		{
			if (r.motor_status(i) != Good)
			{
				faulty = true;
			}
		}
		if (r.has_battery_voltage() && r.battery_voltage() <= 14.3f)
		{
			faulty = true;
		}
		
		drawRobot(p, frame->blue_team(), r.shell(), center, r.angle(), r.ball_sense_status() == HasBall, faulty);
		
		// Highlight the manually controlled robot
		if (manualID == r.shell())
		{
			p.setPen(Qt::green);
			const float r = Robot_Radius + .05;
			p.drawEllipse(center, r, r);
		}
		
		// Robot text
		QPointF textPos = center - rtX * 0.2 - rtY * (Robot_Radius + 0.1);
		BOOST_FOREACH(const DebugText& text, r.text())
		{
			if (text.layer() < 0 || layerVisible(text.layer()))
			{
				p.setPen(qcolor(text.color()));
				drawText(p, textPos, QString::fromStdString(text.text()), false);
				textPos -= rtY * 0.1;
			}
		}
	}

	// Current ball position and velocity
	if (frame->has_ball())
	{
		QPointF pos = qpointf(frame->ball().pos());
		QPointF vel = qpointf(frame->ball().vel());
		
		p.setPen(ballColor);
		p.setBrush(ballColor);
		p.drawEllipse(QRectF(-Ball_Radius + pos.x(), -Ball_Radius + pos.y(),
				Ball_Diameter, Ball_Diameter));
		
		if (!vel.isNull())
		{
			p.drawLine(pos, QPointF(pos.x() + vel.x(), pos.y() + vel.y()));
		}
	}
}

void FieldView::drawText(QPainter &p, QPointF pos, QString text, bool center)
{
	p.save();
	p.translate(pos);
	p.rotate(_textRotation);
	p.scale(0.008, -0.008);
	
	if (center)
	{
		int flags = Qt::AlignHCenter | Qt::AlignVCenter;
		QRectF r = p.boundingRect(QRectF(), flags, text);
		p.drawText(r, flags, text);
	} else {
		p.drawText(QPointF(), text);
	}
	
	p.restore();
}

void FieldView::drawCoords(QPainter& p)
{
	p.setPen(Qt::gray);
	
	// X
	p.drawLine(QPointF(0, 0), QPointF(0.25, 0));
	p.drawLine(QPointF(0.25, 0), QPointF(0.20, -0.05));
	p.drawLine(QPointF(0.25, 0), QPointF(0.20, 0.05));
	drawText(p, QPointF(0.25, 0.1), "+X");
	
	// Y
	p.drawLine(QPointF(0, 0), QPointF(0, 0.25));
	p.drawLine(QPointF(0, 0.25), QPointF(-0.05, 0.20));
	p.drawLine(QPointF(0, 0.25), QPointF(0.05, 0.20));
	drawText(p, QPointF(0.1, 0.25), "+Y");
}

void FieldView::drawField(QPainter& p, const LogFrame *frame)
{
	p.save();
	
	//reset to center
	p.translate(-Floor_Length/2.0, -Floor_Width/2.0);
	
	p.translate(Field_Border, Field_Border);
	
	p.setPen(Qt::white);
	p.setBrush(Qt::NoBrush);
	p.drawRect(QRectF(0, 0, Field_Length, Field_Width));
	
	//set brush alpha to 0
	p.setBrush(QColor(0,130,0, 0));
	
	//reset to center
	p.translate(Field_Length/2.0, Field_Width/2.0);
	
	//centerline
	p.drawLine(QLineF(0, Field_Width/2,0, -Field_Width/2.0));
	
	//center circle
	p.drawEllipse(QRectF(-Field_CenterRadius, -Field_CenterRadius, 
		Field_CenterDiameter, Field_CenterDiameter));
	
	p.translate(-Field_Length/2.0, 0);
	
	//goal areas
	p.drawArc(QRectF(-Field_ArcRadius, -Field_ArcRadius + Field_GoalFlat/2.f, 2.f * Field_ArcRadius, 2.f * Field_ArcRadius), -90*16, 90*16);
	p.drawArc(QRectF(-Field_ArcRadius, -Field_ArcRadius - Field_GoalFlat/2.f, 2.f * Field_ArcRadius, 2.f * Field_ArcRadius), 90*16, -90*16);
	p.drawLine(QLineF(Field_ArcRadius, -Field_GoalFlat/2.f, Field_ArcRadius, Field_GoalFlat/2.f));
	// Penalty Mark
	p.drawEllipse(QRectF(-Field_PenaltyDiam/2.0f + Field_PenaltyDist, -Field_PenaltyDiam/2.0f, Field_PenaltyDiam, Field_PenaltyDiam));
	
	p.translate(Field_Length, 0);
	
	p.drawArc(QRectF(-Field_ArcRadius, -Field_ArcRadius + Field_GoalFlat/2.f, 2.f * Field_ArcRadius, 2.f * Field_ArcRadius), -90*16, -90*16);
	p.drawArc(QRectF(-Field_ArcRadius, -Field_ArcRadius - Field_GoalFlat/2.f, 2.f * Field_ArcRadius, 2.f * Field_ArcRadius), 90*16, 90*16);
	p.drawLine(QLineF(-Field_ArcRadius, -Field_GoalFlat/2.f, -Field_ArcRadius, Field_GoalFlat/2.f));
	// Penalty Mark
	p.drawEllipse(QRectF(-Field_PenaltyDiam/2.0f - Field_PenaltyDist, -Field_PenaltyDiam/2.0f, Field_PenaltyDiam, Field_PenaltyDiam));
		
	// goals
	float x[2] = {0, Field_GoalDepth};
	float y[2] = {Field_GoalWidth/2.0f, -Field_GoalWidth/2.0f};
	
	bool flip = frame->blue_team() ^ frame->defend_plus_x();
	
	p.setPen(flip ? Qt::yellow : Qt::blue);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	
	x[0] -= Field_Length;
	x[1] -= Field_Length + 2 * Field_GoalDepth;
	
	p.setPen(flip ? Qt::blue : Qt::yellow);
	p.drawLine(QLineF(x[0], y[0], x[1], y[0]));
	p.drawLine(QLineF(x[0], y[1], x[1], y[1]));
	p.drawLine(QLineF(x[1], y[1], x[1], y[0]));
	

	p.restore();
}

void FieldView::drawRobot(QPainter& painter, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall, bool faulty)
{
	painter.setPen(Qt::white);
	painter.setBrush(Qt::NoBrush);
	
	painter.save();

	painter.translate(pos.x(), pos.y());
	
	if (faulty)
	{
		painter.setPen(Qt::red);
	} else if (blueRobot)
	{
		painter.setPen(Qt::blue);
		painter.setBrush(Qt::blue);
	} else {
		painter.setPen(Qt::yellow);
		painter.setBrush(Qt::yellow);
	}
	
	painter.rotate(theta * RadiansToDegrees + 90);
	
	int span = 40;
	
	int start = span*16 + 90*16;
	int end = 360*16 - (span*2)*16;
	const float r = Robot_Radius;
	painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);

    if(showDotPatterns)
    {
        painter.setPen(Qt::NoPen);
        for(int i = 0; i < 4; i++)
        {
            painter.setBrush(QBrush(Dot_Pattern_Colors[ID][i]));
            QPointF center;
            center.setX( (i >= 2) ? Dots_Small_Offset : Dots_Large_Offset );
            center.setX( center.x() * ( (i == 1 || i == 2) ? -1 : 1 ) );
            center.setY( (i <= 1) ? Dots_Small_Offset : Dots_Large_Offset );
            center.setY( center.y() * ( (i <= 1) ? -1 : 1 ) );
            painter.drawEllipse(center, Dots_Radius, Dots_Radius);
        }
    }
	
	if (hasBall)
	{
		painter.setPen(Qt::red);
		const float r = Robot_Radius * 0.75f;
		painter.drawChord(QRectF(-r, -r, r * 2, r * 2), start, end);
	}
	
	painter.restore();

	//	draw shell number
	painter.save();
	painter.translate(pos.x(), pos.y());
	if (blueRobot) {
		painter.setPen(Qt::white);
	} else {
		painter.setPen(Qt::black);
	}
	drawText(painter, QPointF(), QString::number(ID));
	painter.restore();
}

void FieldView::resizeEvent(QResizeEvent* e)
{
	int givenW = e->size().width();
	int givenH = e->size().height();
	int needW, needH;
	if (_rotate & 1)
	{
		needH = roundf(givenW * Floor_Length / Floor_Width);
		needW = roundf(givenH * Floor_Width / Floor_Length);
	} else {
		needH = roundf(givenW * Floor_Width / Floor_Length);
		needW = roundf(givenH * Floor_Length / Floor_Width);
	}
	
	QSize size;
	if (needW < givenW)
	{
		size = QSize(needW, givenH);
	} else {
		size = QSize(givenW, needH);
	}
	
	if (size != e->size())
	{
		resize(size);
	}
    e->accept();
}
