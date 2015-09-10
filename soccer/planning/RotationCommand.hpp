#pragma once

#include "Geometry2d/Point.hpp"

namespace Planning {
    struct RotationCommand {
    public:
        enum CommandType { FacePoint, FaceAngle };

        virtual ~RotationCommand() = default;

        CommandType getCommandType() const {
            return commandType;
        }

    protected:
        RotationCommand(CommandType command) : commandType(command) {}

    private:
        const CommandType commandType;
    };

    struct FacePointCommand : public RotationCommand {
        explicit FacePointCommand(Geometry2d::Point target): RotationCommand(FacePoint), targetPos(target) {}

        const Geometry2d::Point targetPos;
    };

    struct FaceAngleCommand : public RotationCommand {
        explicit FaceAngleCommand(float radians): RotationCommand(FaceAngle), targetAngle(radians) {}

        const float targetAngle;
    };
}