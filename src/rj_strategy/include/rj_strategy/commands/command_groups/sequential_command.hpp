#pragma once
#include <initializer_list>
#include <list>

#include "rj_strategy/command.hpp"

namespace commands {

class SequentialCommand : public Command {
public:
    SequentialCommand(std::initializer_list<std::shared_ptr<Command>> commands);

    void initialize() override;
    void execute() override;
    void exit(bool interrupted) override;
    bool is_done() override;

private:
    std::list<std::shared_ptr<Command>> commands;
    std::shared_ptr<std::list<std::shared_ptr<Command>>::iterator> curIt;
    bool finished;
};

}  // namespace commands