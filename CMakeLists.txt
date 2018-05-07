cmake_minimum_required(VERSION 2.8.12)

project(yolo_depth_fusion)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
    roscpp
    darknet_ros_msgs
    image_transport
    cv_bridge
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
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

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
    roscpp
    darknet_ros_msgs
    image_transport
    cv_bridge
  DEPENDS
    Boost
)

add_executable(relay src/relay.cpp)
target_link_libraries(relay ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(relay yolo_depth_fusion_generate_messages_cpp)
