#ifndef _SETUP_MODE_H_
#define _SETUP_MODE_H_

#include <QMouseEvent>
#include <Geometry/Point2d.hpp>

class Setup_Mode
{
public:
	virtual ~Setup_Mode();
	
	virtual void draw();
	virtual void start();
	virtual void ok();
	virtual void cancel();
	
	virtual void mousePressEvent(QMouseEvent *e, Geometry::Point2d pos);
	virtual void mouseMoveEvent(QMouseEvent *e, Geometry::Point2d pos);
	virtual void mouseDoubleClickEvent(QMouseEvent *e, Geometry::Point2d pos);
};

#endif // _SETUP_MODE_H_
