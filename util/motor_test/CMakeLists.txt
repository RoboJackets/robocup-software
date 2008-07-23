project (motor_test)

add_definitions (-Werror)
add_definitions (-DQT_NO_KEYWORDS)

# this command finds the Qt4 libraries and sets all required variables
find_package (Qt4 REQUIRED)

# add some useful macros and variables
# QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ). 
# It contains a path to CMake scripts
include (${QT_USE_FILE})

include_directories (${PROJECT_BINARY_DIR})

# add all source files here
SET (SRCS
	main.cpp Labjack_U3.cpp USB_Device.cpp Test_Window.cpp Test_Table_Model.cpp Test_Interface.cpp
	)

qt4_wrap_cpp (MOC_SRCS
    Test_Window.hpp
    )
	
qt4_wrap_ui (UI_SRCS
    test_window.ui
    )
    
add_executable (${PROJECT_NAME} ${SRCS} ${UI_SRCS} ${MOC_SRCS})

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} usb)
