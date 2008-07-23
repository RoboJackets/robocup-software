#ifndef _DISTORTION_SETUP_H_
#define _DISTORTION_SETUP_H_

#include "GL_Camera_View.h"
#include "Setup_Mode.h"

namespace Vision
{
	class Process;
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
	virtual void mousePressEvent(QMouseEvent *e, Geometry::Point2d pos);
	virtual void mouseMoveEvent(QMouseEvent *e, Geometry::Point2d pos);
	virtual void mouseDoubleClickEvent(QMouseEvent *e, Geometry::Point2d pos);

protected:
	GL_Camera_View *_view;
	
	// Line being drawn, used for calibration lens distortion.
    // When in distortion mode, there is always a calibration line here.
    // The last point is the one moved by the mouse.
    Vision::Calibration_Line *_cal_line;
    
    Vision::Distortion *distortion() const { return _view->vision()->distortion; }

    void draw_cal_line(Vision::Calibration_Line *line);
};

#endif // _DISTORTION_SETUP_H_
