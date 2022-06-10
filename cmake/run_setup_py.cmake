#
# Runs pip install -e . during installation time to install a python package.
#
# run_setup_py(<setup.py_dir> <build_dir> <install_dir>)
#
# Example (Also most common use case):
# run_setup_py(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_INSTALL_PREFIX})
#
function(run_setup_py setup_py_dir build_dir install_dir)
    find_program(PYTHON3 "python3" REQUIRED)

    set(SETUP_PY    "${setup_py_dir}/setup.py")
    set(SETUP_ARGS  "--prefix ${install_dir} --no-deps -e .")
    set(SETUP_COMMAND "${PYTHON3} -m pip install ${SETUP_ARGS} WORKING_DIRECTORY ${setup_py_dir}")

    install(CODE "
    execute_process(COMMAND ${SETUP_COMMAND} OUTPUT_VARIABLE out ERROR_VARIABLE err RESULT_VARIABLE res)
    if (NOT \${res} EQUAL \"0\")
        message(FATAL_ERROR \"out: \${out}, err: \${err}, res: \${res}\")
    endif()")
endfunction()

