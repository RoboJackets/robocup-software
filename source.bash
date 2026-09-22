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

if [[ $SHELL == *"fish"* ]]; then
    echo "fish detected, sourcing fish"
    source /opt/ros/humble/setup.fish
    if [[ -f install/setup.fish ]]; then
        source install/setup.fish
    fi
fi