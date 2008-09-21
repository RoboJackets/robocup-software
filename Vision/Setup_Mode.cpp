#include "Setup_Mode.h"

Setup_Mode::~Setup_Mode()
{
}

void Setup_Mode::draw()
{
}

void Setup_Mode::start()
{
}

void Setup_Mode::ok()
{
}

void Setup_Mode::cancel()
{
}

void Setup_Mode::mousePressEvent(QMouseEvent *e, Geometry::Point2d pos)
{
}

void Setup_Mode::mouseMoveEvent(QMouseEvent *e, Geometry::Point2d pos)
{
}

void Setup_Mode::mouseDoubleClickEvent(QMouseEvent *e, Geometry::Point2d pos)
{
	// Send another click event
	mousePressEvent(e, pos);
}
