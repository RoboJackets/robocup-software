#pragma once
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace commands {

/**
    Meant to be inherited from to make a specific command
*/
class Command {
public:
    Command(std::string name, std::vector<std::string> requirements) {
        this->name = name;
        this->requirements = requirements;
    };

    Command(std::string name) { this->name = name; }

    virtual void initialize(){};
    virtual void execute() {}
    virtual bool is_done() { return true; }
    virtual void exit(bool interrupted) {}

    std::string get_name() { return name; }

    std::vector<std::string> getRequirements() { return requirements; }

    bool is_scheduled() { return scheduled; }

    // WARNING: ONLY COMMAND SCHEDULER SHOULD USE THIS
    void mark_scheduled() { scheduled = true; }

    // WARNING: ONLY COMMAND SCHEDULER SHOULD USE THIS
    void mark_unsheduled() { scheduled = false; }

    bool is_interruptible() { return interruptible; }

    bool requirements_overlap(Command& other) {
        if (this->requirements.size() == 0 || other.requirements.size() == 0) return false;
        for (int i = 0; i < this->requirements.size(); i++) {
            std::string curRobot = this->requirements[i];
            for (int j = 0; i < other.requirements.size(); j++) {
                if (curRobot == other.requirements[j]) {
                    return true;
                }
            }
        }
        return false;
    }

    void addRequirements(std::string requirement) { requirements.push_back(requirement); }

protected:
    std::string name;

private:
    bool interruptible = true;
    bool scheduled = false;
    std::vector<std::string> requirements;
};
}  // namespace commands