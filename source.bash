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

