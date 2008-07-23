#ifndef _ROBOTS_HPP_
#define _ROBOTS_HPP_

#include <stdint.h>
#include <string>
#include <list>

#include <Packet/SkillCmd.hpp>
#include <Packet/SkillStatus.hpp>
#include <Packet/VisionData.hpp>

class Role;
class Opponent;

namespace Tactics
{
    class Base;
}

// Information about one robot on our team.
class Robot
{
public:
    Robot(int id, bool self);

    bool assigned() const;

    // Returns true if vision data for this robot is valid.
    bool visible() const { return _vision->valid; }

    // Sets the motion command for this robot to move to a point.
    void move(const Geometry::Point2d &pt);
    void face(const Geometry::Point2d &pt);
    void kick(float s);

    // If this is one of our robots, this is the role it is assigned (if any).
    Role *role() const { return _role; }
    void role(Role *role);

    // If this is an opponent robot, this is the opponent identity it is assigned.
    Opponent *opponent() const { return _opponent; }
    void opponent(Opponent *opp);

    Tactics::Base *tactic() const;

    bool goalie() const { return _goalie; }
    void goalie(bool flag);

    const Geometry::Point2d &pos() const { return _vision->pos; }

    // Returns the direction the robot is facing as a vector.
    Geometry::Point2d facing() const;

    int id() const { return _id; }
    Packet::SkillCmd::Robot *skill() const { return _skill; }
    const Packet::VisionData::Robot *vision() const { return _vision; }

    const Packet::SkillStatus::Robot *skill_status() const { return _skill_status; }
    Packet::SkillStatus::Status skill_status_code() const;
    void clear_status() { _skill_status->status = Packet::SkillStatus::None; }

    const std::string &name() const { return _name; }
    void name(const std::string &name) { _name = name; }

    // Robots per team
    static const int Num_Robots = 5;

    // Finds a robot by name.
    // Searches self robots first, then opponents.
    static Robot *find(const std::string &name);

    // Finds all robots running a tactic of a given type
    // (or descended from that type).
    template<class T>
    static void find_by_type(std::list<Robot *> &robots)
    {
        for (int i = 0; i < Num_Robots; ++i)
        {
            Robot *r = &self[i];
            Tactics::Base *tactic = r->tactic();
            if (tactic && dynamic_cast<T>(tactic))
            {
                robots.push_back(r);
            }
        }
    }

    void free(bool f) { _free = f; }
    bool free() const { return _free; }

    static Robot self[Num_Robots], opp[Num_Robots];

protected:
    int _id;
    bool _self;
    std::string _name;
    Packet::SkillCmd::Robot *_skill;
    Packet::VisionData::Robot *_vision;
    Packet::SkillStatus::Robot* _skill_status;

    Role *_role;
    Opponent *_opponent;
    bool _goalie;

    /** if true, this robot is not controlled by another robot/tactic */
    bool _free;
};

extern Packet::VisionData vision_packet;
extern Packet::SkillCmd skill_packet;
extern Packet::SkillStatus skill_status_packet;

#endif // _ROBOTS_HPP_
