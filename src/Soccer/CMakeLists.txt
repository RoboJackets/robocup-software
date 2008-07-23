add_definitions(-O2 -Wall -g3)

###
MACRO(ANTLR_SRCS _grammar _srcs_name)
    GET_FILENAME_COMPONENT(_abs_grammar ${_grammar} ABSOLUTE)

    FILE(READ ${_grammar} contents)
    STRING(REGEX MATCHALL "class [^ ]+ extends [^ ]+;" classes "${contents}")
    SET(${_srcs_name})
    SET(_headers)
    FOREACH(match ${classes})
        STRING(REGEX REPLACE "class ([^ ]+).*" "\\1" name ${match})
        SET(${_srcs_name} ${${_srcs_name}} ${CMAKE_CURRENT_BINARY_DIR}/${name}.cpp)
        SET(_headers ${_headers} ${CMAKE_CURRENT_BINARY_DIR}/${name}.hpp)
    ENDFOREACH(match)

    ADD_CUSTOM_COMMAND(OUTPUT ${${_srcs_name}} ${_headers}
        COMMAND cantlr ARGS ${_abs_grammar}
        DEPENDS ${_abs_grammar}
    )
ENDMACRO(ANTLR_SRCS)

antlr_srcs(play.g PARSER_SRCS)
###

project(soccer)

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../common/CMakeLists.txt")

add_definitions (-DQT_NO_KEYWORDS)
set (QT_USE_QTNETWORK true)
find_package (Qt4 REQUIRED)
include (${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(../common)
add_executable(soccer 
	soccer.cpp
    Robot.cpp
    Playbook.cpp
    Play.cpp
    Role.cpp
    Condition.cpp
    Opponent.cpp
    Predicates.cpp
    
    tactics/Tactics.cpp

    #parameters
    tactics/parameters/Parameter.cpp
    tactics/parameters/Point_Parameter.cpp
    tactics/parameters/Float_Parameter.cpp
    tactics/parameters/Bool_Parameter.cpp
    tactics/parameters/Robot_Parameter.cpp

    #tactics
    tactics/Assistant_Goalie.cpp
    tactics/Handler.cpp
    tactics/Forward.cpp
    tactics/Goalie.cpp
    tactics/Midfielder.cpp

    #test tactics
    tactics/skills/Kick.cpp
    tactics/skills/Dribble.cpp
    tactics/skills/Move.cpp
    tactics/skills/Pass.cpp
    tactics/skills/Orient.cpp
    tactics/skills/Steal.cpp
    tactics/skills/Mark.cpp
    tactics/skills/Receive.cpp
    ${PARSER_SRCS}
)

target_link_libraries(soccer antlr boost_filesystem ${PACKET} ${GEOMETRY} ${QT_LIBRARIES})
