if [[ $SHELL == *"bash"* ]]; then
    echo "bash detected, sourcing bash"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
fi

if [[ $SHELL == *"zsh"* ]]; then
    echo "zsh detected, sourcing zsh"
    source /opt/ros/humble/setup.zsh
    source install/setup.zsh
fi
