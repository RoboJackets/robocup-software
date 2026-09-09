#!/usr/bin/env bash
set -euo pipefail

readonly workspace_dir="$(pwd)"

git config --global --add safe.directory "${workspace_dir}"
git submodule sync --recursive
git submodule update --init --recursive

source /opt/ros/humble/setup.bash
make perf_docker

# Automatically source ROS in future VS Code terminals.
readonly bashrc_marker="# RoboCup dev container environment"

if ! grep -Fqx "${bashrc_marker}" "${HOME}/.bashrc"; then
    {
        printf '\n%s\n' "${bashrc_marker}"
        printf 'source /opt/ros/humble/setup.bash\n'
        printf 'if [ -f %q ]; then source %q; fi\n' \
            "${workspace_dir}/install/setup.bash" \
            "${workspace_dir}/install/setup.bash"
    } >> "${HOME}/.bashrc"
fi