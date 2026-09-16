#include "rj_ui/game_settings_cache.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <spdlog/spdlog.h>

namespace game_settings_cache {

namespace {

// Kept in sync with util/run_with_restart.bash, which deletes this file before
// the first launch.
std::filesystem::path cache_path() {
    const char* home = std::getenv("HOME");
    const std::filesystem::path base = home != nullptr ? std::filesystem::path{home} : ".";
    return base / ".robocup" / "last_game_settings.txt";
}

}  // namespace

void save(const GameSettings& settings) {
    const std::filesystem::path path = cache_path();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);

    std::ofstream out{path, std::ios::trunc};
    if (!out) {
        SPDLOG_WARN("Failed to write game settings cache to {}", path.string());
        return;
    }
    out << "request_blue_team " << static_cast<int>(settings.request_blue_team) << '\n'
        << "request_goalie_id " << settings.request_goalie_id << '\n'
        << "defend_plus_x " << static_cast<int>(settings.defend_plus_x) << '\n';
}

void load(GameSettings* settings) {
    std::ifstream in{cache_path()};
    if (!in) {
        return;
    }

    std::string key;
    int value = 0;
    while (in >> key >> value) {
        if (key == "request_blue_team") {
            settings->request_blue_team = value != 0;
        } else if (key == "request_goalie_id") {
            settings->request_goalie_id = value;
        } else if (key == "defend_plus_x") {
            settings->defend_plus_x = value != 0;
        }
    }
}

}  // namespace game_settings_cache
