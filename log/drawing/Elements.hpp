#ifndef ELEMENTS_HPP_
#define ELEMENTS_HPP_

#include <QPainter>

#include <Team.h>
#include <Geometry/Point2d.hpp>

void drawField(QPainter& p);

void drawRobot(QPainter& p, Team t, unsigned char ID, Geometry::Point2d pos, float theta);

void drawBall(QPainter& p, Geometry::Point2d pos);

#endif /* ELEMENTS_HPP_ */
