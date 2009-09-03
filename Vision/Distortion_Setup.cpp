#include "Distortion_Setup.h"
#include "vision/Process.h"
#include "vision/Distortion.h"

#include <boost/foreach.hpp>

using namespace boost;

Distortion_Setup::Distortion_Setup(GL_Camera_View *view)
{
	_view = view;
	_cal_line = 0;
}

void Distortion_Setup::start()
{
	BOOST_FOREACH(Vision::Calibration_Line *line, distortion()->cal_lines)
	{
		delete line;
	}
	distortion()->cal_lines.clear();
	
	_cal_line = new Vision::Calibration_Line;
	_cal_line->points.push_back(_view->last_mouse_pos());
}

void Distortion_Setup::ok()
{
	// Delete the line being drawn
	cancel();

	// Run auto calibration
	distortion()->frame_size(_view->frame_width(), _view->frame_height());
	distortion()->auto_calibrate();
}

void Distortion_Setup::cancel()
{
	if (_cal_line)
	{
		delete _cal_line;
		_cal_line = 0;
	}
}

void Distortion_Setup::draw()
{
	if (_cal_line)
	{
		glPointSize(4);
		glColor3ub(255, 255, 255);
		BOOST_FOREACH(Vision::Calibration_Line *line, distortion()->cal_lines)
		{
			draw_cal_line(line);
		}
		
		glColor3ub(255, 255, 0);
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1, 0x3333);
		if (_cal_line)
		{
			draw_cal_line(_cal_line);
		}
		glDisable(GL_LINE_STIPPLE);
	}
}

void Distortion_Setup::mousePressEvent(QMouseEvent *e, Geometry2d::Point pos)
{
	if (e->button() == Qt::LeftButton)
	{
		_cal_line->points.push_back(pos);
	} else if (e->button() == Qt::RightButton)
	{
		if (_cal_line->points.size() <= 1)
		{
			if (!distortion()->cal_lines.empty())
			{
				Vision::Calibration_Line *last_line = distortion()->cal_lines.back();
				
				// Go back to the last point on the last line
				delete _cal_line;
				_cal_line = last_line;
				
				// Add another point to the line to track the mouse
				_cal_line->points.push_back(pos);
				
				// Remove the line from the transform, as it will be added again when it's finished
				distortion()->cal_lines.pop_back();
			}
		} else {
			// Cancel the last point
			_cal_line->points.pop_back();
			
			// Move the new last point to the mouse position
			_cal_line->points.back() = pos;
		}
	}
	
	_view->updateGL();
}

void Distortion_Setup::mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos)
{
    if (_cal_line && !_cal_line->points.empty())
    {
    	_cal_line->points.back() = pos;
    	_view->updateGL();
    }
}

void Distortion_Setup::mouseDoubleClickEvent(QMouseEvent *e, Geometry2d::Point pos)
{
	if (e->button() == Qt::LeftButton)
	{
		// Require at least 4 points:  2 endpoints, 1 to make it curved, and 1 that
		// is a tracking the mouse and is a duplicate of the last point
		// (added by the first click in this doubleclick).
		if (_cal_line && _cal_line->points.size() >= 4)
		{
			// Remove the last point, which was tracking the mouse
			_cal_line->points.pop_back();
			
			// Move this line to the transform
			distortion()->cal_lines.push_back(_cal_line);
			
			// Start a new line
			_cal_line = new Vision::Calibration_Line;
			_cal_line->points.push_back(_view->last_mouse_pos());
		}
	}
}

void Distortion_Setup::draw_cal_line(Vision::Calibration_Line *line)
{
	glBegin(GL_LINE_STRIP);
	BOOST_FOREACH(const Geometry2d::Point &pt, line->points)
	{
		glVertex2f(pt.x, pt.y);
	}
	glEnd();
	
	glBegin(GL_POINTS);
	BOOST_FOREACH(const Geometry2d::Point &pt, line->points)
	{
		glVertex2f(pt.x, pt.y);
	}
	glEnd();
}
