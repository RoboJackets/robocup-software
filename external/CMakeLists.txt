add_subdirectory(robocup-fshare)
add_subdirectory(rrt)
add_subdirectory(grSim)

#======================================================================
# Packaging
#======================================================================
install(TARGETS grSim
    DESTINATION lib/${CMAKE_PROJECT_NAME})
