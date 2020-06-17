#!/bin/bash

# This script emulates what the colcon generated setup.bash does
# kind of in that it sets the PATH, LD_LIBRARY_PATH and AMENT_PREFIX_PATH
# variables.

_INSTALL_PATH=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

_path_add() {
    if [ -d "$1" ] && [[ ":$PATH:" != *":$1:"* ]]; then
        export PATH="$1${PATH:+":$PATH"}"
    fi
}

_ld_library_path_add() {
    if [ -d "$1" ] && [[ ":$LD_LIBRARY_PATH:" != *":$1:"* ]]; then
        export LD_LIBRARY_PATH="$1${LD_LIBRARY_PATH:+":$LD_LIBRARY_PATH"}"
    fi
}

_ament_prefix_path_add() {
    if [ -d "$1" ] && [[ ":$AMENT_PREFIX_PATH:" != *":$1:"* ]]; then
        export AMENT_PREFIX_PATH="$1${AMENT_PREFIX_PATH:+":$AMENT_PREFIX_PATH"}"
    fi
}

_pythonpath_add() {
    if [ -d "$1" ] && [[ ":$PYTHONPATH:" != *":$1:"* ]]; then
        export PYTHONPATH="$1${PYTHONPATH:+":$PYTHONPATH"}"
    fi
}

_path_add "${_INSTALL_PATH}/bin"
_ld_library_path_add "${_INSTALL_PATH}/lib"
_ament_prefix_path_add "${_INSTALL_PATH}"
_pythonpath_add "${_INSTALL_PATH}/lib/python3.6/site-packages"

unset _pathadd
unset _ld_library_path_add
unset _ament_prefix_path_add

unset _INSTALL_PATH
