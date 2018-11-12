#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <vector>


class CameraBall {
public:
    /**
     * Wrapper to throw out all the extra crap in the Protobuf packet
     *
     * @param timeCaptured Time that the picture was taken
     * @param pos Position of the ball at that time
     */
    CameraBall(RJ::Time timeCaptured, Geometry2d::Point pos)
        : timeCaptured(timeCaptured), pos(pos);

    Geometry2d::Point getPos();

    RJ::Time getTimeCaptured();

    /**
     * Combines all the balls in the list and returns a ball
     * with the average pos and time
     *
     * @param balls The list of balls to combine
     */
    static CameraBall CombineBalls(std::vector<CameraBall> balls);

private:
    Geometry2d::Point pos;
    RJ::Time timeCaptured;
};