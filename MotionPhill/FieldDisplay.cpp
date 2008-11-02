#include "FieldDisplay.hpp"

#include <QPainter>

/*FIXME - Change to use Constants.hpp instead of Sizes.h.*/
#include <Sizes.h>
#include <Team.h>
#include <ui_motion.h>
#include "Graphikos/Field.hpp"
#include "MainWindow.hpp"

using namespace Graphikos;

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
	painter.scale(width()/FLOOR_LENGTH, -height()/FLOOR_WIDTH);

	//draw field from center
	painter.translate(FLOOR_LENGTH/2.0, -FLOOR_WIDTH/2.0);
	Field::paint(painter);

	//now move to team space and draw everything else
	if (_team == Yellow)
	{
		painter.translate(-FIELD_LENGTH/2.0f, 0);
		painter.rotate(-90);
	}
	else
	{
		painter.translate(FIELD_LENGTH/2.0f, 0);
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
	int h = int(w * FEILD_ASPECT);

	if (h > event->size().height())
	{
		h = event->size().height();
		w = int(h/FEILD_ASPECT);
	}

	this->resize(w,h);
	event->accept();
}

void FieldDisplay::mouseReleaseEvent(QMouseEvent* me)
{
      	float wx = me->x() * FLOOR_LENGTH / width();
	float wy = me->y() * FLOOR_WIDTH / height();

	wx -= FLOOR_LENGTH/2.0f;
	wy = FLOOR_WIDTH/2.0f - wy;

	float x = me->y() * FLOOR_WIDTH / height();
	float y = me->x() * FLOOR_LENGTH / width();

	if (_team == Yellow)
	{
		x -= FLOOR_WIDTH/2.0f;
		y -= FIELD_DEADSPACE;
	}
	else
	{
		y = FLOOR_LENGTH - y - FIELD_DEADSPACE;
		x = FLOOR_WIDTH/2.0f - x;
	}
        newPosition(x, y, wx, wy, *me);
}
