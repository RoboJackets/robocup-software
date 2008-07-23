#include "Pass.hpp"

using namespace Packet;

Tactics::Factory_Type<Tactics::Pass> pass_tactic("pass");

Tactics::Pass::Pass(Role *role):
    Base(role),
    _receiver_param(this, "to")
{
}

float Tactics::Pass::score(Robot *robot)
{
    return robot->pos().distTo(vision_packet.ball.pos);
}

void Tactics::Pass::start()
{
    state = State_Get;
}

void Tactics::Pass::run()
{
    const VisionData::Robot *vis = robot()->vision();
    if (!vis->valid || !_receiver_param.valid())
    {
        return;
    }
    
    Packet::SkillCmd::Robot *skill = robot()->skill();
    int seq = robot()->skill_status()->sequence;
    Packet::SkillStatus::Status status = robot()->skill_status_code();
    
    printf("state %d status %d\n", state, status);
    switch (state)
    {
        case State_Get:
            if (status == Packet::SkillStatus::Done && seq == state)
            {
                state = State_Orient;
            }
            break;
    
        case State_Orient:
            if (status == Packet::SkillStatus::Done && seq == state)
            {
                state = State_Kick;
                
                Geometry::Point2d start = vision_packet.ball.pos;
                Geometry::Point2d end = _receiver_param.robot()->pos();
                _threshold.pt[0] = (start + end) / 2;
                _threshold.pt[1] = _threshold.pt[0] + (end - start).perpCCW();
            }
            break;
        
        case State_Kick:
            if (vision_packet.ball.vel.mag() > 0.8)
            {
                state = State_Moving;
            }
            break;
        
        case State_Moving:
        {
            if (vision_packet.ball.vel.mag() < 0.5)
            {
                // Ball is stopping.  Need to kick again.
                state = State_Get;
            }
            
            Geometry::Point2d v0 = vision_packet.ball.pos - _threshold.pt[0];
            Geometry::Point2d v1 = _threshold.pt[1] - _threshold.pt[0];
            float x = v0.x * v1.y - v0.y * v1.x;
            printf("x %g\n", x);
            if (x > 0)
            {
                state = State_Done;
            }
            
            break;
        }
        
        default:
            break;
    }
    
    switch (state)
    {
        case State_Get:
        {
            skill->valid = true;
            skill->skill = Packet::SkillCmd::GotoBall;
            skill->motion.face = _receiver_param.robot()->pos();
            break;
        }
        
        case State_Orient:
        {
            skill->valid = true;
            skill->skill = Packet::SkillCmd::Orient;
            skill->motion.face = _receiver_param.robot()->pos();
            break;
        }
        
        case State_Kick:
        {
            skill->valid = true;
            skill->skill = Packet::SkillCmd::Kick;
            skill->motion.kick = 80;
            break;
        }
                
        default:
            break;
    }
    skill->sequence = state;
}

bool Tactics::Pass::done()
{
    return state == State_Done;
}
