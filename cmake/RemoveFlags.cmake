# Removes a CXX flag.
# Usage:
#   remove_cxx_flag(<flag>)
macro(REMOVE_CXX_FLAG flag)
  string(REPLACE "${flag}" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endmacro()


# Removes a C flag.
# Usage:
#   remove_c_flag(<flag>)
macro(REMOVE_C_FLAG flag)
  string(REPLACE "${flag}" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
endmacro()
