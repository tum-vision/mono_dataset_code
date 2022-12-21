INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()

add_executable(fractal_create fractal_create.cpp)
add_executable(fractal_tracker fractal_tracker.cpp)
add_executable(fractal_print_marker fractal_print_marker.cpp)
add_executable(fractal_pix2meters fractal_pix2meters.cpp)

target_link_libraries(fractal_create		aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(fractal_tracker		aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(fractal_print_marker	aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(fractal_pix2meters	aruco opencv_calib3d opencv_highgui ${THREADLIB})

INSTALL(TARGETS fractal_tracker RUNTIME DESTINATION bin)
