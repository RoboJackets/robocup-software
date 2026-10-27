#!/bin/bash
#
# Supervisor that keeps the soccer stack alive across crashes.
#
# Persistence itself is enabled by the auto_restart:=True launch argument (see
# soccer.launch.py), which tells the UI to persist the operator's team color /
# goalie / side to a cache file and reload it on startup. This script owns that
# cache's lifecycle: it deletes the cache before the FIRST launch so a fresh
# session starts from CLI args, and leaves it in place for every restart after
# that.
#
# Usage:
#   util/run_with_restart.bash ros2 launch ./launch/soccer.launch.py auto_restart:=True <args...>

set -u

# Keep in sync with game_settings_cache_path() in
# src/rj_config_server/src/config_server.cpp.
GAME_SETTINGS_CACHE="${HOME}/.robocup/last_game_settings.txt"

RESTART_DELAY="${RJ_RESTART_DELAY:-1}"

if [[ $# -eq 0 ]]; then
    echo "usage: $0 <launch command...>" >&2
    exit 1
fi

# Fresh session: drop any stale cache
rm -f "${GAME_SETTINGS_CACHE}"

# Track whether the operator asked us to stop (Ctrl-C / SIGTERM).
should_stop=0
launch_pid=""

on_signal() {
    should_stop=1
    if [[ -n "${launch_pid}" ]]; then
        kill -INT "${launch_pid}" 2>/dev/null
    fi
}
trap on_signal INT TERM

while true; do
    "$@" &
    launch_pid=$!
    wait "${launch_pid}"
    exit_code=$?
    launch_pid=""

    if [[ "${should_stop}" -eq 1 ]]; then
        echo "[run_with_restart] shutdown requested, exiting (last exit code ${exit_code})."
        exit "${exit_code}"
    fi

    echo "[run_with_restart] stack crashed (exit code ${exit_code}), restarting in ${RESTART_DELAY}s..."
    sleep "${RESTART_DELAY}"
done
