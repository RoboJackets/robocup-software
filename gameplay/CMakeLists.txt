project (GameplayModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

###
# FIXME - This has been known to fork-bomb if "make -j" is used without a thread limit.
#         Always use e.g. "make -j 3"
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
        COMMAND cantlr.sh ARGS ${_abs_grammar}
        DEPENDS ${_abs_grammar}
    )
ENDMACRO(ANTLR_SRCS)

antlr_srcs(play.g PARSER_SRCS)
###

qt4_wrap_cpp ( MOC_SRCS
)

set (SRCS
    GameplayModule.cpp
    
    Playbook.cpp
    Robot.cpp
    Play.cpp
    Behavior.cpp
    Opponent.cpp
    Play_Lexer.cpp
    Play_Parser.cpp
    Condition.cpp
    Role.cpp
    Predicates.cpp
    GraphNode.cpp
    Window.cpp
	GameplayUtils.cpp
    
    parameters/Parameter.cpp
    parameters/Bool_Parameter.cpp
    parameters/Point_Parameter.cpp
    parameters/Robot_Parameter.cpp
    parameters/Float_Parameter.cpp
    parameters/String_Parameter.cpp
    
    behaviors/Move.cpp
    behaviors/Kickoff.cpp
    behaviors/ReceivePass.cpp
    behaviors/Penalty.cpp
    behaviors/Idle.cpp
    behaviors/Kick.cpp
    behaviors/Mark.cpp
    behaviors/Steal.cpp
    behaviors/Intercept.cpp
	behaviors/Dribble.cpp
	
    behaviors/positions/Goalie.cpp	
    behaviors/positions/Fullback.cpp
    behaviors/positions/Halfback.cpp
    behaviors/positions/Striker.cpp	
    behaviors/positions/Forward.cpp
    
 	behaviors/test/KickerCalibration.cpp   
    behaviors/test/MotionLeader.cpp    
    behaviors/test/Pivot.cpp
)

add_library (gameplay SHARED ${SRCS} ${MOC_SRCS})
target_link_libraries (gameplay ${QT_LIBRARIES})

add_dependencies(gameplay ${FRAMEWORK_LIBRARY})
