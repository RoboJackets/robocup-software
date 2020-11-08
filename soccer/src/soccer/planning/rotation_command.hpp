#pragma once

#include <sstream>
#include <string>

#include "rj_geometry/point.hpp"

namespace Planning {
struct RotationCommand {
public:
    enum CommandType { FacePoint, FaceAngle, None };

    virtual ~RotationCommand() = default;

    CommandType get_command_type() const { return command_type_; }

    [[nodiscard]] virtual std::string print() const = 0;

    friend std::ostream& operator<<(std::ostream& stream,
                                    const RotationCommand& rot_cmd) {
        stream << rot_cmd.print();
        return stream;
    }

protected:
    RotationCommand(CommandType command) : command_type_(command) {}

private:
    const CommandType command_type_;
};

struct FacePointCommand : public RotationCommand {
    explicit FacePointCommand(rj_geometry::Point target)
        : RotationCommand(FacePoint), target_pos(target) {}

    const rj_geometry::Point target_pos;

    [[nodiscard]] std::string print() const override {
        std::stringstream ss;
        ss << "FacePointCommand(" << target_pos << ")";
        return ss.str();
    }
};

struct FaceAngleCommand : public RotationCommand {
    explicit FaceAngleCommand(float radians)
        : RotationCommand(FaceAngle), target_angle(radians) {}

    const float target_angle;

    [[nodiscard]] std::string print() const override {
        std::stringstream ss;
        ss << "FaceAngleCommand(" << target_angle << ")";
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
