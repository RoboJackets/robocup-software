#pragma once

#include <sstream>
#include <string>

#include "Geometry2d/Point.hpp"

namespace Planning {
struct RotationCommand {
public:
    enum CommandType { FacePoint, FaceAngle, None };

    virtual ~RotationCommand() = default;

    CommandType getCommandType() const { return commandType; }

    [[nodiscard]] virtual std::string print() const = 0;

    friend std::ostream& operator<<(std::ostream& stream,
                                    const RotationCommand& rot_cmd) {
        stream << rot_cmd.print();
        return stream;
    }

protected:
    RotationCommand(CommandType command) : commandType(command) {}

private:
    const CommandType commandType;
};

struct FacePointCommand : public RotationCommand {
    explicit FacePointCommand(Geometry2d::Point target)
        : RotationCommand(FacePoint), targetPos(target) {}

    const Geometry2d::Point targetPos;

    [[nodiscard]] std::string print() const override {
        std::stringstream ss;
        ss << "FacePointCommand(" << targetPos << ")";
        return ss.str();
    }
};

struct FaceAngleCommand : public RotationCommand {
    explicit FaceAngleCommand(float radians)
        : RotationCommand(FaceAngle), targetAngle(radians) {}

    const float targetAngle;

    [[nodiscard]] std::string print() const override {
        std::stringstream ss;
        ss << "FaceAngleCommand(" << targetAngle << ")";
        return ss.str();
    }
};

struct EmptyAngleCommand : public RotationCommand {
    EmptyAngleCommand() : RotationCommand(None) {}

    [[nodiscard]] std::string print() const override {
        return "EmptyAngleCommand()";
    }
};
}  // namespace Planning
