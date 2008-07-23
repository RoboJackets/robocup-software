#include "Robot.hpp"
#include "Role.hpp"
#include "Opponent.hpp"

using namespace Geometry;
using namespace Packet;

Robot Robot::self[5] =
{
    Robot(0, true),
    Robot(1, true),
    Robot(2, true),
    Robot(3, true),
    Robot(4, true)
};

Robot Robot::opp[5] =
{
    Robot(0, false),
    Robot(1, false),
    Robot(2, false),
    Robot(3, false),
    Robot(4, false)
};

Packet::SkillCmd skill_packet;
Packet::SkillStatus skill_status_packet;
Packet::VisionData vision_packet;

Robot::Robot(int id, bool self) :
	_free(true)
{
    _id = id;
    _self = self;
    _role = 0;
    _opponent = 0;
    _goalie = false;
    
    if (self)
    {
    	_skill_status = &skill_status_packet.robots[id];
        _skill = &skill_packet.robots[id];
        _vision = &vision_packet.self[id];
    } else {
        _skill = 0;
        _vision = &vision_packet.opp[id];
    }
}

bool Robot::assigned() const
{
    return (_role && _role->assigned()) || _opponent || _goalie;
}

void Robot::move(const Geometry::Point2d &pt)
{
	_skill->valid = true;
	_skill->motion.pos = pt;
}

void Robot::face(const Geometry::Point2d &pt)
{
	_skill->valid = true;
    _skill->motion.face = pt;
}

void Robot::kick(float s)
{
	//TODO kick is a skill, you give a point to kick to
	//_skill->valid = true;
    //_skill->_motion.kick = (int)roundf(s * 8);
}

void Robot::role(Role *role)
{
    _role = role;
    
    if (_role)
    {
        _name = role->name();
    } else {
        _name.clear();
    }
}

void Robot::opponent(Opponent *opp)
{
    _opponent = opp;
    
    if (_opponent)
    {
        _name = opp->name();
    } else {
        _name.clear();
    }
}

Tactics::Base *Robot::tactic() const
{
    if (_role)
    {
        return _role->current_tactic();
    } else {
        return 0;
    }
}

void Robot::goalie(bool flag)
{
	_goalie = flag;
	if (flag)
	{
		_name = "goalie";
	} else {
		_name.clear();
	}
}

Robot *Robot::find(const std::string &name)
{
    for (int i = 0; i < 5; ++i)
    {
        if (self[i].name() == name)
        {
            return &self[i];
        }
    }

    for (int i = 0; i < 5; ++i)
    {
        if (opp[i].name() == name)
        {
            return &opp[i];
        }
    }
    
    return 0;
}

Geometry::Point2d Robot::facing() const
{
    return Geometry::Point2d::direction(_vision->theta * M_PI / 180.0);
}

Packet::SkillStatus::Status Robot::skill_status_code() const
{
    if (_skill_status)
    {
        return _skill_status->status;
    } else {
        return Packet::SkillStatus::None;
    }
}
