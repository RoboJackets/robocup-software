#pragma once

#include "GL_Camera_View.h"
#include "Setup_Mode.h"
#include "vision/Process.h"

namespace Vision
{
	class Calibration_Line;
	class Transform;
}

class Distortion_Setup: public Setup_Mode
{
public:
	Distortion_Setup(GL_Camera_View *view);
	
	virtual void start();
	virtual void ok();
	virtual void cancel();
	virtual void draw();
	virtual void mousePressEvent(QMouseEvent *e, Geometry2d::Point pos);
	virtual void mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos);
	virtual void mouseDoubleClickEvent(QMouseEvent *e, Geometry2d::Point pos);

protected:
	GL_Camera_View *_view;
	
	// Line being drawn, used for calibration lens distortion.
    // When in distortion mode, there is always a calibration line here.
    // The last point is the one moved by the mouse.
    Vision::Calibration_Line *_cal_line;
    
    Vision::Distortion *distortion() const { return _view->vision()->distortion; }

    void draw_cal_line(Vision::Calibration_Line *line);
};
