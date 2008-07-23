#include "Midfielder.hpp"
#include "../Role.hpp"


#include "../Predicates.hpp"
#include "parameters/Robot_Parameter.hpp"

#include <Sizes.h>
#include <Geometry/Line2d.hpp>

using namespace Geometry;
using namespace Packet;

typedef struct
{
    Point2d center;
    int num_robots;
    int score;
} Zone;

Tactics::Factory_Type<Tactics::Midfielder> midfielder("midfielder");

Tactics::Midfielder::Midfielder(Role *role) :
    Base(role),
    opp_param(this, "opp")
{
}

void Tactics::Midfielder::run()
{
    if(robot()->free() && running)
    {
        printf("running midfielder\n");
        const int num_forward_zones = 4;
        const float area_radius = 10.0;
        Point2d robot_pos = robot()->pos();
        Point2d dest;
        Robot* r = robot();

        Zone forward_zones[4];
	Zone target_zone;

	forward_zones[0].center.x = robot_pos.x + 10;
        forward_zones[0].center.y = robot_pos.y + 10;
        forward_zones[1].center.x = robot_pos.x + 20;
        forward_zones[1].center.y = robot_pos.y + 10;
        forward_zones[0].center.x = robot_pos.x - 10;
        forward_zones[0].center.y = robot_pos.y + 10;
        forward_zones[0].center.x = robot_pos.x - 20;
        forward_zones[0].center.y = robot_pos.y + 20;

	for(int i = 0; i<=5; i++)
        {
            if(Robot::opp[i].visible())
            {
                for(int j = 0; j<=num_forward_zones; j++)
                {
                    if(Robot::opp[i].pos().inThreshold(forward_zones[j].center,area_radius))
                    {
                        forward_zones[j].num_robots++;
                    }
                }
            }
        }

        target_zone = forward_zones[0];
        for(int i = 0; i<=num_forward_zones; i++)
        {
            if(forward_zones[i].num_robots < target_zone.num_robots)
            {
                target_zone =  forward_zones[i];
            }
	}

        dest  = target_zone.center;

        Packet::SkillCmd::Robot& skill = *r->skill();
	skill.valid = true;
	skill.skill = Packet::SkillCmd::None;
	skill.motion.style = MotionCmd::Fast;

	r->move(dest);
    }


}

