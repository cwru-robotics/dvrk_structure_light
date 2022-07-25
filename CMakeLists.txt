cmake_minimum_required(VERSION 2.8.3)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} HAVE_DSHOW HAVE_VIDEOINPUT" CACHE STRING "Default CXX options" FORCE)
# message("CMake flags: " ${CMAKE_CXX_FLAGS}) 

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} HAVE_DSHOW HAVE_VIDEOINPUT" CACHE STRING "Default C options" FORCE)
# message("CMake flags: " ${CMAKE_C_FLAGS}) 

project(dvrk_structure_light)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	image_transport
	sensor_msgs
)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport sensor_msgs)

# find_package( Gphoto2 REQUIRED )
find_package(OpenCV)

# ${GPHOTO2_INCLUDE_DIR}
include_directories(
	${catkin_INCLUDE_DIRS}
	${catkin_INCLUDES}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(pattern_display_test src/pattern_display_test.cpp)
add_executable(image_subscriber_test src/image_subscriber_test.cpp)
add_executable(gray_code_pattern_out_test src/gray_code_pattern_out_test.cpp)
add_executable(gray_code_pattern_capture src/gray_code_pattern_capture.cpp)
add_executable(gray_code_pattern_decode src/gray_code_pattern_decode.cpp)
add_executable(stereo_gray_code_pattern_capture src/stereo_gray_code_pattern_capture.cpp)
add_executable(stereo_gray_code_pattern_decode src/stereo_gray_code_pattern_decode.cpp)
add_executable(structured_light_combined src/structured_light_combined.cpp)
add_executable(single_camera_position_1 src/single_camera_position_1.cpp)
add_executable(single_camera_position_2 src/single_camera_position_2.cpp)
add_executable(single_camera_decode src/single_camera_decode.cpp)
add_executable(lau_calibration_less src/lau_calibration_less.cpp)

add_custom_command(
	TARGET test_nncam PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_CURRENT_SOURCE_DIR}/lib/libnncam.so
		${CATKIN_DEVEL_PREFIX}/lib/libnncam.so
)

target_link_libraries(pattern_display_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(image_subscriber_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern_out_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern_decode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(stereo_gray_code_pattern_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(stereo_gray_code_pattern_decode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(structured_light_combined ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(single_camera_position_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(single_camera_position_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(single_camera_decode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lau_calibration_less ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
