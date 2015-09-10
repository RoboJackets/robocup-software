#pragma once

#include "Geometry2d/Point.hpp"
#include "MotionConstraints.hpp"
#include "RotationPlanner.hpp"

namespace Planning {
    struct RotationCommand {
    public:
        virtual ~RotationCommand() = default;
        enum CommandType { FacePoint, FaceAngle };
        virtual std::shared_ptr<RotationPlanner> getDefaultPlanner() const {
            return nullptr;
        };

        virtual CommandType getCommandType() const = 0;
    protected:
        RotationCommand() {}
    };

    struct FacePointCommand : public RotationCommand {
        explicit FacePointCommand(Geometry2d::Point target): targetPos(target) {}
        virtual CommandType getCommandType() const override {
            return RotationCommand::FacePoint;
        }
        const Geometry2d::Point targetPos;
    };

    struct FaceAngleCommand : public RotationCommand {
        explicit FaceAngleCommand(float radians): targetAngle(radians) {}
        virtual CommandType getCommandType() const override {
            return RotationCommand::FaceAngle;
        }
        const float targetAngle;
    };

    struct RotationConstraints {
    public:
        RotationConstraints() : maxSpeed(*MotionConstraints::_max_rotation_speed),
                        maxAccel(*MotionConstraints::_max_rotation_acceleration) { }
        float maxSpeed;
        float maxAccel;
    };
}