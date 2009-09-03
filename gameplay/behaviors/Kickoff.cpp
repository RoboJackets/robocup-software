#include "Kickoff.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Kickoff> behavior("kickoff");

Gameplay::Behaviors::Kickoff::Kickoff(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role)
{
}

void Gameplay::Behaviors::Kickoff::run()
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
            break;

        default:
            break;
    }
}

bool Gameplay::Behaviors::Kickoff::done()
{
    return gameplay()->state()->gameState.state == GameState::Playing;
}
