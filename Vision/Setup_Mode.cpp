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

void Setup_Mode::mousePressEvent(QMouseEvent *e, Geometry2d::Point pos)
{
}

void Setup_Mode::mouseMoveEvent(QMouseEvent *e, Geometry2d::Point pos)
{
}

void Setup_Mode::mouseDoubleClickEvent(QMouseEvent *e, Geometry2d::Point pos)
{
	// Send another click event
	mousePressEvent(e, pos);
}
