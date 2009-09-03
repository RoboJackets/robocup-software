#include "Transform_Setup.h"
#include "vision/Transform.h"

#include <GL/glut.h>
#include <QMessageBox>

using namespace Eigen;

void vertex(Geometry2d::Point pt)
{
	glVertex2f(pt.x, pt.y);
}

void arrow(Geometry2d::Point p0, Geometry2d::Point p1, float head_size)
{
	glBegin(GL_LINES);
	    vertex(p0);
	    vertex(p1);
	
	    Geometry2d::Point delta = (p1 - p0).normalized() * head_size;
	    Geometry2d::Point perp(delta.y / 2, -delta.x / 2);
	    vertex(p1);
	    vertex(p1 + perp - delta);
	
	    vertex(p1);
	    vertex(p1 - perp - delta);
	glEnd();
}

Transform_Setup::Transform_Setup(GL_Camera_View *view)
{
	_view = view;
	_point = 0;
	transform = 0;
}

void Transform_Setup::start()
{
	_point = 0;
	
	for (int i = 0; i < 3; ++i)
	{
		transform->image_point[i] = _view->last_mouse_pos();
	}
	
	_view->updateGL();
}

void Transform_Setup::ok()
{
	if (!transform->calculate_matrix(_view->vision()->distortion))
	{
		QMessageBox::critical(_view, "Set Transformation", "Reference points must not be colinear.");
	}
}

void Transform_Setup::cancel()
{
}

void Transform_Setup::draw()
{
	static uint8_t point_color[3][3] =
	{
		{255, 255, 255},
		{255, 64, 64},
		{64, 255, 64}
	};
	
	float r = 10 / _view->scale;
	glColor3ub(255, 255, 255);
	glBegin(GL_LINES);
		glVertex2f(transform->image_point[0].x - r, transform->image_point[0].y - r);
		glVertex2f(transform->image_point[0].x + r, transform->image_point[0].y + r);

		glVertex2f(transform->image_point[0].x - r, transform->image_point[0].y + r);
		glVertex2f(transform->image_point[0].x + r, transform->image_point[0].y - r);
	glEnd();
	
	if (_point > 0)
	{
		glColor3ub(255, 32, 32);
		arrow(transform->image_point[0], transform->image_point[1], 10 / _view->scale);
	}

	if (_point > 1)
	{
		glColor3ub(32, 255, 32);
		arrow(transform->image_point[0], transform->image_point[2], 10 / _view->scale);
	}
	
	for (int i = 0; i <= _point; ++i)
	{
		char ch = '0' + i;
		
		glColor3ubv(point_color[i]);

		glPushMatrix();
		glTranslatef(transform->image_point[i].x, transform->image_point[i].y, 0);
		
		glScalef(0.1 / _view->scale, -0.1 / _view->scale, 1);
        glTranslatef(-200, 200, 0);
        glutStrokeCharacter(GLUT_STROKE_ROMAN, ch);
        glPopMatrix();
	}
}

void Transform_Setup::mousePressEvent(QMouseEvent *e, Geometry2d::Point pos)
{
	if (e->button() == Qt::LeftButton)
	{
		transform->image_point[_point] = pos;
		
		++_point;
		if (_point <= 2)
		{
			// Start placing the next point
			transform->image_point[_point] = pos;
		} else {
			// Done with all three points
			_view->setup_ok();
		}
	} else if (e->button() == Qt::RightButton && _point > 0)
	{
		--_point;
		transform->image_point[_point] = pos;
	}
}

void Transform_Setup::mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos)
{
	transform->image_point[_point] = pos;
	_view->updateGL();
}
