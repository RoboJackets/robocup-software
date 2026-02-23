sanitize_prefix_path() {
    local value="$1"
    local cleaned=""
    local entry
    while IFS= read -r entry; do
        if [[ -d "$entry" ]]; then
            if [[ -z "$cleaned" ]]; then
                cleaned="$entry"
            else
                cleaned="${cleaned}:$entry"
            fi
        fi
    done < <(printf '%s' "$value" | tr ':' '\n')
    printf '%s' "$cleaned"
}

if [[ $SHELL == *"bash"* ]]; then
    echo "bash detected, sourcing bash"
    source /opt/ros/humble/setup.bash
    if [[ -f install/setup.bash ]]; then
        source install/setup.bash
    fi
fi

if [[ $SHELL == *"zsh"* ]]; then
    echo "zsh detected, sourcing zsh"
    source /opt/ros/humble/setup.zsh
    if [[ -f install/setup.zsh ]]; then
        source install/setup.zsh
    fi
fi

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    export AMENT_PREFIX_PATH="$(sanitize_prefix_path "${AMENT_PREFIX_PATH}")"
fi
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    export CMAKE_PREFIX_PATH="$(sanitize_prefix_path "${CMAKE_PREFIX_PATH}")"
fi
