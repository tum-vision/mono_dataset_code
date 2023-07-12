#------------------------------------------------------
# Build type
#------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE "Debug" )
endif()

#------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------

if(WIN32)
    # Postfix of DLLs:
    # Postfix of DLLs:
    set(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    set(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")

else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
#    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/ /usr/lib/cmake)
endif()

option(USE_OWN_EIGEN3	"Set to OFF to use a standard eigen3 version" ON)
option(BUILD_UTILS	"Set to OFF to not compile utils " ON)
option(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)
option(BUILD_GLSAMPLES 	"Set to OFF to build static libraries" OFF)
option(USE_TIMERS 	"Set to OFF to build static libraries" OFF)
option(BUILD_DEBPACKAGE 	"Set to ON to use cpack" OFF)
option(ARUCO_DEVINSTALL 	"Set to OFF to disable source installation" ON)
  iF(USE_TIMERS)
add_definitions(-DUSE_TIMERS)
ENDIF()
# ----------------------------------------------------------------------------
#   PROJECT CONFIGURATION
#   force some variables that could be defined in the command line to be written to cache
# ----------------------------------------------------------------------------
option(INSTALL_DOC 	"Set to ON to build/install Documentation" OFF)
if (INSTALL_DOC)
    find_package(Doxygen REQUIRED)
    message( STATUS "INSTALL_DOC:         ${INSTALL_DOC} ")
    include("${PROJECT_SOURCE_DIR}/cmake/generateDoc.cmake")
    generate_documentation(${PROJECT_SOURCE_DIR}/cmake/dox.in)
endif()

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file("${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")
