add_subdirectory(robocup-fshare)
add_subdirectory(rrt)
add_subdirectory(grSim)
add_subdirectory(erSim EXCLUDE_FROM_ALL)

#======================================================================
# Packaging
#======================================================================
install(TARGETS simulator-cli DESTINATION lib/${CMAKE_PROJECT_NAME})
