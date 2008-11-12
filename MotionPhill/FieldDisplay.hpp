#ifndef FIELDDISPLAY_HPP_
#define FIELDDISPLAY_HPP_

#include <QWidget>
#include <QResizeEvent>
#include <QMouseEvent>

#include <Geometry/Point2d.hpp>
#include <Team.h>

#include "RobotPath.hpp"

class FieldDisplay: public QWidget
{
	Q_OBJECT;

	public:
		FieldDisplay(QWidget *parent = 0);
                void setTeam(Team& t);
                void setRobotPath(RobotPath &rp);

	Q_SIGNALS:
		void newPosition(float x, float y, float wx, float wy, QMouseEvent me);

	protected:
		void paintEvent(QPaintEvent* pe);
		void resizeEvent(QResizeEvent* re);
		void mouseReleaseEvent(QMouseEvent* me);

        private:
                Team _team;
                RobotPath _rp;

};

#endif /*FIELDDISPLAY_HPP_*/
