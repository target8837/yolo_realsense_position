cmake_minimum_required(VERSION 2.8.12)

project(yolo_depth_fusion)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
    roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_message_files(
  FILES
    yoloObject.msg
    yoloObjects.msg
)


generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

install(DIRECTORY include/${PROJECT_NAME})

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
    roscpp
)

add_executable(relay_fake src/relay_fake.cpp)
target_link_libraries(relay_fake ${catkin_LIBRARIES})
add_dependencies(relay_fake yolo_depth_fusion_generate_messages_cpp)
