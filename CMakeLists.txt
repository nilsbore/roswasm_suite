cmake_minimum_required(VERSION 2.8.3)
project(roscpp_json_serialize)

# Build the talker and listener. Each one uses the following ROS packages,
# which we need to find_package() individually:
#   roscpp (the client library)
#   std_msgs (contains the std_msgs/String message type)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosmon_msgs REQUIRED)

# We've found them; now use their variables in the usual way to configure
# the compile and link steps.
# Note: we skip calling link_directories() because ROS packages follow the
# recommended CMake practice of returning absolute paths to libraries
include_directories(include external/rapidjson/include ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${rosmon_msgs_INCLUDE_DIRS})

add_executable(test src/test.cpp)
target_link_libraries(test ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${rosmon_msgs_LIBRARIES})

# (optional) Install the executables.
install(TARGETS test
        DESTINATION bin)

