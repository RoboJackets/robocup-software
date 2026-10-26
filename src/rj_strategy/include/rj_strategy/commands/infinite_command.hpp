#pragma once

#include "rj_strategy/command.hpp"

namespace commands {

/*
An infinite command that does nothing
*/

class InfiniteCommand : public Command {
public:
    InfiniteCommand();
    bool is_done() override;
    void execute() override;
};

}  // namespace commands