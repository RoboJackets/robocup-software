if [ $SHELL = '/usr/bin/bash' ]; then
    echo "bash detected, sourcing bash"
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
fi

if [ $SHELL = '/usr/bin/zsh' ]; then
    echo "zsh detected, sourcing zsh"
    source /opt/ros/foxy/setup.zsh
    source install/setup.zsh
fi
