#pragma once
class Robot {
public:
    Robot(int id);

    int getId();
    bool equals(Robot other);

private:
    int id;
};