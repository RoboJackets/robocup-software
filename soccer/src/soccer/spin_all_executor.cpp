#include "spin_all_executor.hpp"

namespace rj {

void SpinAllExecutor::spin_all() {
    auto start = std::chrono::steady_clock::now();
    auto max_duration_not_elapsed = [max_duration, start]() {
        if (std::chrono::nanoseconds(0) == max_duration) {
            // told to spin forever if need be
            return true;
        } else if (std::chrono::steady_clock::now() - start < max_duration) {
            // told to spin only for some maximum amount of time
            return true;
        }
        // spun too long
        return false;
    };

    if (spinning.exchange(true)) {
        throw std::runtime_error("spin_some() called while already spinning");
    }
    RCLCPP_SCOPE_EXIT(this->spinning.store(false););
    bool work_available = false;
    while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
        AnyExecutable any_exec;
        if (!work_available) {
            wait_for_work(std::chrono::milliseconds::zero());
        }
        if (get_next_ready_executable(any_exec)) {
            execute_any_executable(any_exec);
            work_available = true;
        } else {
            if (!work_available) {
                break;
            }
            work_available = false;
        }
    }
}

}  // namespace rj
