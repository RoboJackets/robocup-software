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
    if [[ ":$PYTHONPATH:" != *":$1:"* ]]; then
        export PYTHONPATH="$1${PYTHONPATH:+":$PYTHONPATH"}"
    fi
}

# NOTE: Distutils is deprecated and scheduled to be removed in Python3.12.
# No direct replacement for the command exists, and in Ubuntu 22.04,
# this script will produce a warning about this. We may need to find a
# replacement library in the future.
_PYTHON_LIB_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(prefix='${_INSTALL_PATH}'))")
_pythonpath_add "${_PYTHON_LIB_PATH}"
unset _PYTHON_LIB_PATH

_path_add "${_INSTALL_PATH}/bin"
_ld_library_path_add "${_INSTALL_PATH}/lib"
_ament_prefix_path_add "${_INSTALL_PATH}"

unset _pathadd
unset _ld_library_path_add
unset _ament_prefix_path_add

unset _INSTALL_PATH
