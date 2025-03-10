if [[ $SHELL == *"bash"* ]]; then
    echo "bash detected, sourcing bash"
    source ~/tracing_ws/install/setup.bash
    source install/setup.bash
fi

if [[ $SHELL == *"zsh"* ]]; then
    echo "zsh detected, sourcing zsh"
    source /opt/ros/humble/setup.zsh
    source install/setup.zsh
fi

source install/env.sh

