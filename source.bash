# Welcome to our Robocup Setup Bash Script :)

# Check if user already sourced
if [[ -n "$AMENT_PREFIX_PATH" ]]; then
    echo "WARNING: a workspace is already sourced in this shell." >&2
    echo "  If you just deleted install or make clean, open a new terminal " >&2
    echo "  stale paths cause PackageNotFoundError at launch." >&2
fi

# Pin compiler to GCC 11 to match /opt/ros/humble binaries
if [[ -x /usr/bin/g++-11 && -x /usr/bin/gcc-11 ]]; then
    export CC=/usr/bin/gcc-11
    export CXX=/usr/bin/g++-11
else
    echo "ERROR: gcc-11/g++-11 not found. Install with: " >&2
    echo "  sudo apt install gcc-11 g++-11" >&2
    return 1
fi

# Running ROS Setup with shell detection
if [[ -n "$BASH_VERSION" ]]; then
    source /opt/ros/humble/setup.bash
    if [[ -f install/setup.bash ]]; then
        source install/setup.bash
    fi
elif [[ -n "$ZSH_VERSION" ]]; then
    source /opt/ros/humble/setup.zsh
    if [[ -f install/setup.zsh ]]; then
        source install/setup.zsh
    fi
else
    echo "ERROR: unsupported shell. This script needs bash or zsh." >&2
    return 1
fi

echo "ROS Setup Complete!"

