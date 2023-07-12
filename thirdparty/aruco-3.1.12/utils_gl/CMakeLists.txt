INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
add_executable(aruco_test_gl aruco_test_gl.cpp)
add_executable(aruco_test_markermap_gl aruco_test_markermap_gl.cpp)

target_link_libraries(aruco_test_gl aruco ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY} opencv_imgproc opencv_highgui opencv_videoio)
target_link_libraries(aruco_test_markermap_gl aruco ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY} opencv_calib3d)

#add_executable(aruco_test_board_gl_mask aruco_test_board_gl_mask.cpp)
#target_link_libraries(aruco_test_board_gl_mask ${OPENGL_LIBRARIES})

install(TARGETS aruco_test_gl aruco_test_markermap_gl   RUNTIME DESTINATION bin)
