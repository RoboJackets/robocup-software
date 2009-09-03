#pragma once

#include "Setup_Mode.h"
#include "GL_Camera_View.h"

#include <eigen/matrix.h>

class Transform_Setup: public Setup_Mode
{
public:
	Transform_Setup(GL_Camera_View *view);
	
	virtual void start();
	virtual void ok();
	virtual void cancel();
	virtual void draw();
	virtual void mousePressEvent(QMouseEvent *e, Geometry2d::Point pos);
	virtual void mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos);

    Vision::Transform *transform;
	
protected:
	GL_Camera_View *_view;
	int _point;
};
