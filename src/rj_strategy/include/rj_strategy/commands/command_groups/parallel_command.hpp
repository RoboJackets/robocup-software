#pragma once
#include <initializer_list>
#include <list>
#include <utility>

#include "rj_strategy/command.hpp"

namespace commands {

/**
Commands in a parallel command MUST NOT have the same requirements.
If they do share requirements, the constructor will throw an error.
 */
class ParallelCommand : public Command {
public:
    ParallelCommand(std::initializer_list<std::shared_ptr<Command>> commands);

    void initialize() override;
    void execute() override;
    void exit(bool interrupted) override;
    bool is_done() override;

private:
    std::list<std::pair<bool, std::shared_ptr<Command>>> commandPairs;
    int finishedCommands;
};
}  // namespace commands