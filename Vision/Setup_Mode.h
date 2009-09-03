#pragma once

#include <QMouseEvent>
#include <Geometry2d/Point.hpp>

class Setup_Mode
{
public:
	virtual ~Setup_Mode();
	
	virtual void draw();
	virtual void start();
	virtual void ok();
	virtual void cancel();
	
	virtual void mousePressEvent(QMouseEvent *e, Geometry2d::Point pos);
	virtual void mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos);
	virtual void mouseDoubleClickEvent(QMouseEvent *e, Geometry2d::Point pos);
};
