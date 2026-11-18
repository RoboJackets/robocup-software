#pragma once

#include <chrono>
#include <cstdint>
#include <ostream>
#include <string>

using namespace std::chrono_literals;

namespace RJ {

using Time = std::chrono::system_clock::time_point;
using Seconds = std::chrono::duration<double>;

template <class Duration>
constexpr int64_t num_microseconds(Duration d) {
    return std::chrono::duration_cast<std::chrono::microseconds>(d).count();
}

inline Time now() { return std::chrono::system_clock::now(); }

template <class Duration>
constexpr double num_seconds(Duration d) {
    return std::chrono::duration<double>(d).count();
}

}  // namespace RJ

inline RJ::Time operator+(const RJ::Time& time, const RJ::Seconds& sec) {
    return time + std::chrono::duration_cast<RJ::Time::duration>(sec);
}

inline RJ::Time operator-(const RJ::Time& time, const RJ::Seconds& sec) {
    return time - std::chrono::duration_cast<RJ::Time::duration>(sec);
}

inline std::string to_string(RJ::Seconds seconds) {
    return std::to_string(seconds.count()) + "(Seconds)";
}

inline std::ostream& operator<<(std::ostream& os, RJ::Seconds seconds) {
    os << to_string(seconds);
    return os;
}
