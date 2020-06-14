#!/bin/bash

# This script emulates what the colcon generated setup.bash does
# kind of in that it sets the PATH, LD_LIBRARY_PATH and AMENT_PREFIX_PATH
# variables.

_INSTALL_PATH_DIRTY="$(pwd)/$(dirname "$0")"
_INSTALL_PATH="$(readlink -f "${_INSTALL_PATH_DIRTY}")"
unset _INSTALL_PATH_DIRTY

_path_add() {
    if [ -d "$1" ] && [[ ":$PATH:" != *":$1:"* ]]; then
        PATH="$1${PATH:+":$PATH"}"
    fi
}

_ld_library_path_add() {
    if [ -d "$1" ] && [[ ":$LD_LIBRARY_PATH:" != *":$1:"* ]]; then
        LD_LIBRARY_PATH="$1${LD_LIBRARY_PATH:+":$LD_LIBRARY_PATH"}"
    fi
}

_ament_prefix_path_add() {
    if [ -d "$1" ] && [[ ":$AMENT_PREFIX_PATH:" != *":$1:"* ]]; then
        AMENT_PREFIX_PATH="$1${AMENT_PREFIX_PATH:+":$AMENT_PREFIX_PATH"}"
    fi
}

_path_add "${_INSTALL_PATH}/bin"
_ld_library_path_add "${_INSTALL_PATH}/lib"
_ament_prefix_path_add "${_INSTALL_PATH}"

unset _pathadd
unset _ld_library_path_add
unset _ament_prefix_path_add

unset _INSTALL_PATH
