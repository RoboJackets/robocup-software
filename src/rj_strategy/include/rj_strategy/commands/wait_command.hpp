#pragma once
#include <chrono>

#include "rj_strategy/command.hpp"

namespace commands {

class WaitCommand : public Command {
public:
    WaitCommand(int durationMillis);
    ~WaitCommand();
    void initialize() override;
    bool is_done() override;
    void exit(bool interrupted) override;

private:
    std::chrono::time_point<std::chrono::system_clock> startTime;
    std::chrono::milliseconds* duration;
};
}  // namespace commands