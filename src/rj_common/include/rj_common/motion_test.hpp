#pragma once

enum class MotionTestType : int {
    NONE = 0,
    YELLOW_STRAIGHT = 1,
    BLUE_STRAIGHT = 2,
    BORDER_FIELD = 3,
    DEFAULT_FIELD = 4,
};

inline MotionTestType motion_test_type_from_int(int value) {
    switch (value) {
        case static_cast<int>(MotionTestType::YELLOW_STRAIGHT):
            return MotionTestType::YELLOW_STRAIGHT;
        case static_cast<int>(MotionTestType::BLUE_STRAIGHT):
            return MotionTestType::BLUE_STRAIGHT;
        case static_cast<int>(MotionTestType::BORDER_FIELD):
            return MotionTestType::BORDER_FIELD;
        case static_cast<int>(MotionTestType::DEFAULT_FIELD):
            return MotionTestType::DEFAULT_FIELD;
        default:
            return MotionTestType::NONE;
    }
}
