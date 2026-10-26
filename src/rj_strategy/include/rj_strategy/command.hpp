#pragma once
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "rj_strategy/robot.hpp"

namespace commands {

/**
    Meant to be inherited from to make a specific command
*/
class Command {
public:
    Command(std::string name, std::vector<Robot> requirements) {
        this->name = name;
        this->requirements = requirements;
    };

    virtual void initialize(){};
    virtual void execute() {}
    virtual bool is_done() { return true; }
    virtual void exit(bool interrupted) {}

    std::string get_name() { return name; }

    bool is_scheduled() { return scheduled; }

    // WARNING: ONLY COMMAND SCHEDULER SHOULD USE THIS
    void mark_scheduled() { scheduled = true; }

    // WARNING: ONLY COMMAND SCHEDULER SHOULD USE THIS
    void mark_unsheduled() { scheduled = false; }

    bool is_interruptible() { return interruptible; }

    bool requirements_overlap(Command& other) {
        if (this->requirements.size() == 0 || other.requirements.size() == 0) return false;
        for (int i = 0; i < this->requirements.size(); i++) {
            Robot curRobot = this->requirements[i];
            for (int j = 0; i < other.requirements.size(); j++) {
                if (curRobot.equals(other.requirements[j])) {
                    return true;
                }
            }
        }
        return false;
    }

protected:
    std::string name;
    std::vector<Robot> getRequirements() { return requirements; }

private:
    bool interruptible = true;
    bool scheduled = false;
    std::vector<Robot> requirements;
};
}  // namespace commands