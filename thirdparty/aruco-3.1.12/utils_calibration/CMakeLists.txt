INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()
add_executable(aruco_calibration aruco_calibration.cpp calibrator.h calibrator.cpp )
add_executable(aruco_calibration_fromimages aruco_calibration_fromimages.cpp dirreader.h)

target_link_libraries(aruco_calibration             aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(aruco_calibration_fromimages  aruco opencv_calib3d opencv_highgui ${THREADLIB})

install(TARGETS      aruco_calibration aruco_calibration_fromimages  RUNTIME DESTINATION bin)

IF(WIN32)
install(FILES "${PROJECT_SOURCE_DIR}/utils_calibration/aruco_calibration_grid_board_a4.pdf"  DESTINATION bin/)
#install(FILES "${PROJECT_SOURCE_DIR}/utils/myown.dict"   DESTINATION bin/)

elseif(UNIX)
    install(FILES "aruco_calibration_grid_board_a4.pdf"  DESTINATION share/${PROJECT_NAME})
   #install(FILES "${PROJECT_SOURCE_DIR}/utils/myown.dict"   DESTINATION  shared/${PROJECT_NAME})
ENDIF()

