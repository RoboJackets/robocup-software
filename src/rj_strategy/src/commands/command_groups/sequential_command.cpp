#include "rj_strategy/commands/command_groups/sequential_command.hpp"

namespace commands {

SequentialCommand::SequentialCommand(std::initializer_list<std::shared_ptr<Command>> commands)
    : Command("sequential") {
    for (auto command : commands) {
        for (auto curReq : command->getRequirements()) {
            addRequirements(curReq);
        }
        this->commands.push_back(command);
    }
    curIt = std::make_shared<std::list<std::shared_ptr<Command>>::iterator>(this->commands.begin());
}

void SequentialCommand::initialize() {
    curIt = std::make_shared<std::list<std::shared_ptr<Command>>::iterator>(commands.begin());
    (***curIt).initialize();
    finished = false;
}

void SequentialCommand::execute() {
    (***curIt).execute();
    if ((***curIt).is_done()) {
        (***curIt).exit(false);
        (*curIt)++;
        if (*curIt == commands.end()) {
            finished = true;
        } else {
            (***curIt).initialize();
        }
    }
}

bool SequentialCommand::is_done() { return finished; }

void SequentialCommand::exit(bool interrupted) {
    if (interrupted) {
        (***curIt).exit(true);
    }
}

}  // namespace commands