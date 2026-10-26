#include "rj_strategy/commands/infinite_command.hpp"

namespace commands {

InfiniteCommand::InfiniteCommand() : Command("Infinite_Command", {}) {}

bool InfiniteCommand::is_done() { return false; }

void InfiniteCommand::execute() {
    SPDLOG_INFO(
        "==================================================== Infinite Command Execute "
        "==========================================");
}

}  // namespace commands