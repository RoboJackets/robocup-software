#include "rj_strategy/commands/command_groups/parallel_command.hpp"

namespace commands {

ParallelCommand::ParallelCommand(std::initializer_list<std::shared_ptr<Command>> commands)
    : Command("parallel Command") {
    int passes = 1;
    SPDLOG_INFO("=================================Req size {} ===========================",
                getRequirements().size());
    for (auto& command : commands) {
        SPDLOG_INFO("=======================Parallel Cmd Processing {} ====================",
                    command->get_name());
        if (this->requirements_overlap(command)) {
            SPDLOG_ERROR(
                "========================Parallel Command Requirements Overlap!!!!! Pass "
                "{}=================",
                passes);
            throw 505;
        }
        for (auto& curReq : command->getRequirements()) {
            addRequirements(curReq);
        }
        this->commandPairs.push_back(std::make_pair(false, command));
        passes++;
    }
}

void ParallelCommand::initialize() {
    for (auto& pair : commandPairs) {
        pair.first = false;
        pair.second->initialize();
    }
}

void ParallelCommand::execute() {
    for (auto& pair : commandPairs) {
        if (!pair.first) {
            pair.second->execute();
            if (pair.second->is_done()) {
                pair.first = true;
                pair.second->exit(false);
                finishedCommands++;
            }
        }
    }
}

bool ParallelCommand::is_done() { return finishedCommands == commandPairs.size(); }

void ParallelCommand::exit(bool interrupted) {
    if (interrupted) {
        for (auto& pair : commandPairs) {
            if (!pair.first) {
                pair.second->exit(true);
            }
        }
    }
}
}  // namespace commands