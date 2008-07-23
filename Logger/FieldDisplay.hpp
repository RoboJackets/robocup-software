#ifndef FIELDDISPLAY_HPP_
#define FIELDDISPLAY_HPP_

#include <QWidget>
#include <QResizeEvent>
#include <QMouseEvent>

#include "Log/Packet.hpp"

#include <Geometry/Point2d.hpp>

class FieldDisplay: public QWidget
{
	Q_OBJECT;
	
	public:
		FieldDisplay(Team team, QWidget *parent = 0);
		
		void setPackets(QList<Log::LogPacket*>& packets);
		
	Q_SIGNALS:
		void newPosition(float x, float y, float wx, float wy, QMouseEvent me);
		
	protected:
		void paintEvent(QPaintEvent* pe);
		void resizeEvent(QResizeEvent* re);
		void mouseReleaseEvent(QMouseEvent* me);
		
	private:
		QList<Log::LogPacket*> _packets;
		
		Team _team;
};

#endif /*FIELDDISPLAY_HPP_*/
