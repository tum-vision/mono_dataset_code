INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()

add_executable(aruco_dcf aruco_dcf.cpp)
add_executable(aruco_dcf_mm aruco_dcf_markermap.cpp)

target_link_libraries(aruco_dcf		aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(aruco_dcf_mm		aruco opencv_calib3d opencv_highgui ${THREADLIB})

INSTALL(TARGETS aruco_dcf aruco_dcf_mm RUNTIME DESTINATION bin)
