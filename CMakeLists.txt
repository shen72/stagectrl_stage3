cmake_minimum_required(VERSION 2.8.3)
project("stagectrl_stage3")
find_package(catkin REQUIRED)
find_package(stage REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

message(WARNING "stage include dir:" ${STAGE_INCLUDE_DIRS})
message(WARNING "stage link dir: " ${STAGE_LIBRARIES})
include_directories(include ${STAGE_INCLUDE_DIRS})
link_directories("/opt/ros/kinetic/lib")

catkin_package(CATKIN_DEPENDS roscpp std_msgs) 

add_library(basic basic.cc)
target_link_libraries(basic stage)
