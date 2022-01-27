cmake_minimum_required(VERSION 2.8.3)
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

add_executable(pattern_display src/pattern_display.cpp)
add_executable(image_subscriber src/image_subscriber.cpp)
add_executable(gray_code_pattern src/gray_code_pattern.cpp)
add_executable(gray_code_pattern_out src/gray_code_pattern_out.cpp)
add_executable(gray_code_pattern_capture src/gray_code_pattern_capture.cpp)

target_link_libraries(pattern_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern_out ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_code_pattern_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(gray_code_pattern ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GPHOTO2_INCLUDE_DIR)
