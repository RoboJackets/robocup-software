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

    bool requirements_overlap(std::shared_ptr<Command> other) {
        if (this->requirements.size() == 0 || other->getRequirements().size() == 0) return false;
        SPDLOG_INFO("=====================Comparing Command {} with Command {}=================",
                    name, other->get_name());
        for (int i = 0; i < this->requirements.size(); i++) {
            SPDLOG_INFO("=====================Line 1======================");
            std::string curReq = this->requirements.at(i);
            SPDLOG_INFO("=====================Line 2======================");
            for (int j = 0; j < other->getRequirements().size(); j++) {
                SPDLOG_INFO("=====================Line 3======================");
                if (curReq.compare(other->getRequirements().at(j)) == 0) {
                    SPDLOG_INFO(
                        "=======================Requirements overlap: {} AND {}============",
                        curReq, other->getRequirements().at(j));
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