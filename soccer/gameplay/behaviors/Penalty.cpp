#include "Penalty.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Penalty> behavior("penalty");

Gameplay::Behaviors::Penalty::Penalty(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role), _kick(gameplay,role)
{
}


void Gameplay::Behaviors::Penalty::start()
{
    _kick.robot(robot());
    _kick.start();
}
void Gameplay::Behaviors::Penalty::run()
{
    Geometry2d::Point ball(0, Constants::Field::Length / 2);
    if (gameplay()->state()->ball.valid)
    {
        ball = gameplay()->state()->ball.pos;
    }

    GameState::State state = gameplay()->state()->gameState.state;
    switch (state)
    {
        case GameState::Setup:
            robot()->move(Geometry2d::Point(0,Constants::Field::Length / 2 - 0.3));
            robot()->face(ball);
            break;

        case GameState::Ready:
            robot()->move(ball + Geometry2d::Point(0, 0.1));
            robot()->face(ball);
            robot()->willKick = true;
            robot()->state()->radioTx.kick = 255;
            _kick.mode_param.set("normal");
            _kick.run();
            break;

        default:
            robot()->state()->cmd.goalOrientation = ball;
            break;
    }
}

bool Gameplay::Behaviors::Penalty::done()
{
    return gameplay()->state()->gameState.state == GameState::Playing;
}
