cmake_minimum_required(VERSION 2.8.3)
project(dvrk_structure_light)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	image_transport
	sensor_msgs
)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport sensor_msgs)

find_package(OpenCV)

include_directories(
	${catkin_INCLUDE_DIRS}
	${catkin_INCLUDES}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(pattern_display src/pattern_display.cpp)

target_link_libraries(pattern_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
