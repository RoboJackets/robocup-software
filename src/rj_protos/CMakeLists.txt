cmake_minimum_required(VERSION 3.20)
project(rj_protos LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

set(GC_PROTO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/ssl-game-controller/proto)
set(SIM_PROTO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/ssl-simulation-protocol/proto)
set(SIM_PROTO_ROOT ${CMAKE_CURRENT_BINARY_DIR}/sim_proto)
set(GEN_ROOT ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(GEN_DIR ${GEN_ROOT}/rj_protos)

set(GC_PROTOS
  state/ssl_gc_common.proto
  state/ssl_gc_game_event.proto
  state/ssl_gc_referee_message.proto
  geom/ssl_gc_geometry.proto
  vision/ssl_vision_detection.proto
  vision/ssl_vision_geometry.proto
  vision/ssl_vision_wrapper.proto
)

set(SIM_PROTOS
  ssl_simulation_config.proto
  ssl_simulation_control.proto
  ssl_simulation_error.proto
  ssl_simulation_robot_control.proto
  ssl_simulation_robot_feedback.proto
  ssl_simulation_synchronous.proto
)

# There are duplicate protos in ssl-game-controller and ssl-simulation-protocol.
# In order to avoid duplicates, we rename the imports in ssl-simulation-protocol
# to point to the ssl-game-controller ones instead.
set(SIM_IMPORT_REWRITES
  "ssl_gc_common.proto" "state/ssl_gc_common.proto"
  "ssl_vision_detection.proto" "vision/ssl_vision_detection.proto"
  "ssl_vision_geometry.proto" "vision/ssl_vision_geometry.proto"
)

foreach(proto ${SIM_PROTOS})
  file(READ ${SIM_PROTO_SRC}/${proto} content)
  set(rewrites ${SIM_IMPORT_REWRITES})
  while(rewrites)
    list(POP_FRONT rewrites from to)
    string(REPLACE "import \"${from}\";" "import \"${to}\";" content "${content}")
  endwhile()
  file(CONFIGURE OUTPUT ${SIM_PROTO_ROOT}/${proto} CONTENT "${content}" @ONLY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${SIM_PROTO_SRC}/${proto})
endforeach()

set(PROTO_FILES)
foreach(proto ${GC_PROTOS})
  list(APPEND PROTO_FILES ${GC_PROTO_ROOT}/${proto})
endforeach()
foreach(proto ${SIM_PROTOS})
  list(APPEND PROTO_FILES ${SIM_PROTO_ROOT}/${proto})
endforeach()

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(proto ${GC_PROTOS} ${SIM_PROTOS})
  string(REGEX REPLACE "\\.proto$" "" proto_base ${proto})
  set(proto_src ${GEN_DIR}/${proto_base}.pb.cc)
  set(proto_hdr ${GEN_DIR}/${proto_base}.pb.h)
  if(proto IN_LIST GC_PROTOS)
    set(proto_file ${GC_PROTO_ROOT}/${proto})
  else()
    set(proto_file ${SIM_PROTO_ROOT}/${proto})
  endif()
  add_custom_command(
    OUTPUT ${proto_src} ${proto_hdr}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_DIR}
    COMMAND protobuf::protoc
      --cpp_out=${GEN_DIR}
      -I ${GC_PROTO_ROOT}
      -I ${SIM_PROTO_ROOT}
      ${proto_file}
    DEPENDS ${PROTO_FILES} protobuf::protoc
    COMMENT "Running C++ protocol buffer compiler on ${proto}"
    VERBATIM
  )
  list(APPEND PROTO_SRCS ${proto_src})
  list(APPEND PROTO_HDRS ${proto_hdr})
endforeach()

add_library(${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${GEN_ROOT}>
    $<BUILD_INTERFACE:${GEN_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/rj_protos>
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)

install(
  DIRECTORY ${GEN_DIR}
  DESTINATION include
  FILES_MATCHING PATTERN "*.pb.h"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)
ament_package()
