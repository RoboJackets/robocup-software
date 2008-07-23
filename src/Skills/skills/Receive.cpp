#include "Receive.hpp"

#include <stdio.h>

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

Receive::Receive() :
	Skill(Packet::SkillCmd::Receive)
{
}

void Receive::start()
{
}

Packet::MotionCmd::Robot Receive::run()
{
    Packet::MotionCmd::Robot cmd;
    
    cmd.valid = true;
    cmd.face = robot()->vision().ball.pos;
    cmd.roller = 40;
    
    Geometry::Point2d robot_pos = robot()->self().pos;
    Geometry::Point2d ball_pos = robot()->vision().ball.pos;
    Geometry::Point2d ball_vel = robot()->vision().ball.vel;
    Geometry::Point2d ball_vel_norm = ball_vel.norm();
    
    if (ball_vel.mag() > 0.1)
    {
        cmd.pos = ball_pos + ball_vel_norm * (robot_pos - ball_pos).dot(ball_vel_norm);
    } else if (!robot()->status().ballPresent)
    {
        _gotoBall.robot(robot());
        _gotoBall.setCmd(_cmd);
        cmd = _gotoBall.run();
    } else {
        cmd.pos = robot_pos;
    }
    
    if (robot()->status().ballPresent)
    {
        _status = SkillStatus::Done;
    }
    
    return cmd;
}
