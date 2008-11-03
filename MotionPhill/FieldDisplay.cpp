#include "FieldDisplay.hpp"
#include <QPainter>
#include <Constants.hpp>
#include <Team.h>
#include <ui_motion.h>
#include "Graphikos/Field.hpp"
#include "MainWindow.hpp"

using namespace Graphikos;
using namespace Constants;

FieldDisplay::FieldDisplay(QWidget *parent) :
	QWidget(parent), _team(Blue)
{
	this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
}

/*
void FieldDisplay::setPackets(QList<Log::LogPacket*>& packets)
{
    Q_FOREACH(Log::LogPacket *packet, _packets)
    {
        delete packet;
    }

    _packets = packets;
}
*/

void FieldDisplay::setTeam(Team& t)
{
    _team = t;
}

void FieldDisplay::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.scale(width()/Floor::Length, -height()/Floor::Width);

	//draw field from center
	painter.translate(Floor::Length/2.0, -Floor::Width/2.0);
	Graphikos::Field::paint(painter);

	//now move to team space and draw everything else
	if (_team == Yellow)
	{
		painter.translate(-Constants::Field::Length/2.0f, 0);
		painter.rotate(-90);
	}
	else
	{
		painter.translate(Constants::Field::Length/2.0f, 0);
		painter.rotate(90);
	}

        /*
	for (unsigned int i=0; i<_packets.size() ; ++i)
	{
		Log::LogPacket* lp = _packets[i];
		lp->display(painter, _team);
	}
        */
}

void FieldDisplay::resizeEvent(QResizeEvent* event)
{
        int w = event->size().width();
	int h = int(w * Floor::Aspect);

	if (h > event->size().height())
	{
		h = event->size().height();
		w = int(h/Floor::Aspect);
	}

	this->resize(w,h);
	event->accept();
}

void FieldDisplay::mouseReleaseEvent(QMouseEvent* me)
{
      	float wx = me->x() * Floor::Length / width();
	float wy = me->y() * Floor::Width / height();

	wx -= Floor::Length/2.0f;
	wy = Floor::Width/2.0f - wy;

	float x = me->y() * Floor::Width / height();
	float y = me->x() * Floor::Length / width();

	if (_team == Yellow)
	{
		x -= Floor::Width/2.0f;
		y -= Constants::Field::Border;
	}
	else
	{
		y = Floor::Length - y - Constants::Field::Border;
		x = Floor::Width/2.0f - x;
	}
        newPosition(x, y, wx, wy, *me);
}
