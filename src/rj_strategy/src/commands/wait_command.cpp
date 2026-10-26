#include "rj_strategy/commands/wait_command.hpp"

#include <iostream>

#include <spdlog/spdlog.h>

namespace commands {
WaitCommand::WaitCommand(int durationMillis) : Command("Wait Command", {}) {
    this->duration = new std::chrono::milliseconds(durationMillis);
    SPDLOG_INFO(
        "=========================================================================================="
        "==================================================================");
}
WaitCommand::~WaitCommand() { delete duration; }

void WaitCommand::initialize() {
    SPDLOG_INFO(
        "========================================= Wait Command First Line "
        "============================================================");
    startTime = std::chrono::system_clock::now();
    SPDLOG_INFO("Wait Time {}", (*duration).count());
}

bool WaitCommand::is_done() {
    using namespace std::chrono;
    auto curDuration = duration_cast<milliseconds>((system_clock::now() - startTime));
    // SPDLOG_INFO("========================Checking Wait Command Done Cur Duration Millis:
    // {}/{}===================", curDuration.count(), (*duration).count());

    return curDuration >= *duration;
}
void WaitCommand::exit(bool interrupted) {
    SPDLOG_INFO("================================Wait Command Finished======================");
}
}  // namespace commands