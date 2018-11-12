#include <Geometry2d/Point.hpp>
#include <Utils.hpp>


class CameraBall {
public:
    CameraBall(RJ::Time timeCaptured, Geometry2d::Point pos)
        : timeCaptured(timeCaptured), pos(pos);

    Geometry2d::Point pos;
    RJ::Time timeCaptured;
};