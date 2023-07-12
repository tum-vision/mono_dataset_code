# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)
project(aruco VERSION "3.1.12" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

include(cmake/options.cmake)

include(cmake/findDependencies.cmake)
include(cmake/compilerOptions.cmake)



add_subdirectory(src)

if(GL_FOUND AND BUILD_GLSAMPLES)
  add_subdirectory(utils_gl)
endif()
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()


if(BUILD_UTILS)
  add_subdirectory(utils)
  add_subdirectory(utils_markermap)
  add_subdirectory(utils_calibration)
  add_subdirectory(utils_dcf)
  IF(BUILD_SVM)
  ADD_SUBDIRECTORY(utils_svm)
  ENDIF()
   ADD_SUBDIRECTORY(utils_fractal)
endif()
include(cmake/printInfo.cmake)
include(cmake/installOptions.cmake)

# Package Generator  #######################################################
IF(BUILD_DEBPACKAGE)
include(cmake/cpack.cmake)
ENDIF()
