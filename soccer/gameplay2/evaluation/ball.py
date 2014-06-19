

def is_moving_towards_our_goal():
    raise NotImplementedError()
    # Here's the old C++ code from Goalie.cpp:
    # bool Gameplay::Behaviors::Goalie::ballIsMovingTowardsGoal()
    # {
    #     Segment goalLine(Point(-MaxX, 0), Point(MaxX, 0));
    #     Segment ballPath(ball().pos, (ball().pos + 5*ball().vel.normalized()));
    #     //if the ball is traveling towards the goal
    #     return (ball().vel.magsq() > 0.02 && ballPath.intersects(goalLine));
    # }

def is_in_our_goalie_box():
    raise NotImplementedError()


# returns a Robot or None indicating which opponent has the ball
def opponent_with_ball():
    raise NotImplementedError()
    # Here's the old c++ code from Goalie.cpp
    # Robot* Gameplay::Behaviors::Goalie::opponentWithBall()
    # {
    #     Robot* closest = 0;
    #     float bestDist = 0;
    #     BOOST_FOREACH(Robot *r, state()->opp){
    #         float dist = r->pos.distTo(ball().pos);
    #         if (!closest || dist < bestDist)
    #         {
    #             closest = r;
    #             bestDist = dist;
    #         }
    #     }
    #     float angle = 3.14 * (closest->angle / 180);
    #     float theta = angleBetweenThreePoints(Point(closest->pos.x + cos(angle), closest->pos.y + sin(angle)),ball().pos,closest->pos);
    #     float maxRadius = Robot_Diameter + Robot_Diameter*cos(theta);
    #     //std::cout << theta << std::endl;
    #     if(closest && closest->pos.nearPoint(ball().pos, maxRadius))
    #         return closest;
    #     else
    #         return 0;
    # }
