#pragma once

#include <QPainter>

#include <Team.h>
#include <Geometry2d/Point.hpp>

void drawRobot(QPainter& p, Team t, int ID, Geometry2d::Point pos, float theta, Team viewTeam, bool haveBall = false);

void drawBall(QPainter& p, Geometry2d::Point pos, Geometry2d::Point vel = Geometry2d::Point());
