#pragma once

#include <rj_common/game_settings.hpp>

/**
 * @brief Persists the operator's UI-entered game settings so an auto-restarted
 * stack can restore them.
 *
 * The UI is the source of truth for game settings: it seeds them (from CLI args
 * or, when auto-restarting, this cache), then pushes them to the config server.
 * So persistence lives here, not in the config server -- the config server is
 * set correctly on the first tick once the UI pushes the restored values.
 *
 * Only the operator-chosen values are cached: team color (request_blue_team),
 * goalie id (request_goalie_id), and defend side (defend_plus_x).
 *
 * The file lifecycle is owned by util/run_with_restart.bash, which deletes the
 * cache before the first launch so a fresh session starts from CLI args.
 */
namespace game_settings_cache {

/**
 * @brief Write the cached fields of @p settings to disk. No-op-safe on failure.
 */
void save(const GameSettings& settings);

/**
 * @brief Overlay the cached fields onto @p settings if a cache file exists.
 * Leaves @p settings untouched when there is no cache.
 */
void load(GameSettings* settings);

}  // namespace game_settings_cache
