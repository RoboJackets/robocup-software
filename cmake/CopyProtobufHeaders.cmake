# \brief This function copies the generated headers from protobuf_generate_cpp
# to the correct include directory, so that it can be easily installed / added
# as an include directory.
#
# DST_DIR: <The directory to copy headers into>
# OUT_DST_NAMES: <Variable that will be set to a list of the copied file's
#                 paths>
# <ARGN>...: <The files (generated headers) to copy into DST_DIR>
#
# Usage:
#     protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
#     copy_protobuf_headers("include/${PROJECT_NAME}"
#         COPIED_PROTO_HDRS ${PROTO_HDRS})
#
# All relative paths are interpreted to be relative to CMAKE_CURRENT_BINARY_DIR.
#
# The generated headers should be added as a dependency of a library, otherwise
# the command will not do anything.
function(copy_protobuf_headers DST_DIR OUT_DST_NAMES)
    # Check arguments
    if (ARGC LESS 3)
        message(
            FATAL_ERROR "copy_protobuf_headers() requires at least 1 input file.")
    endif ()

    # Normalize the path
    file(TO_CMAKE_PATH "${DST_DIR}" DST_DIR)

    # Get a list of the names of all the output files after copying
    set(proto_hdr_dst "")
    foreach (proto_hdr IN LISTS ARGN)
        get_filename_component(proto_hdr_name ${proto_hdr} NAME)
        get_filename_component(
            proto_hdr_dst_path
            ${DST_DIR}/${proto_hdr_name}
            ABSOLUTE
            BASE_DIR
            ${CMAKE_CURRENT_BINARY_DIR})
        list(APPEND proto_hdr_dst ${proto_hdr_dst_path})
    endforeach ()

    # Create the directory
    add_custom_target(create_proto_include_dir
            COMMAND ${CMAKE_COMMAND} -E make_directory ${DST_DIR})
    # Copy the headers into the directory
    add_custom_command(
        OUTPUT ${proto_hdr_dst}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ARGN} ${DST_DIR}
        DEPENDS ${ARGN} create_proto_include_dir
        VERBATIM)

    set(${OUT_DST_NAMES}
        ${proto_hdr_dst}
        PARENT_SCOPE)
endfunction()
