cmake_minimum_required(VERSION 2.8.3)
project(roswasm_tutorials)

option(ROSWASM_NATIVE "Compile with ROS Native version of ROSWASM" OFF)

if (DEFINED ENV{EMSDK} OR ${ROSWASM_NATIVE})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roswasm std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roswasm_tutorials
#  CATKIN_DEPENDS roscpp roswasm std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(timers src/timers.cpp)
add_executable(bool_client src/bool_client.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"

if (NOT ${ROSWASM_NATIVE})
set_target_properties(talker PROPERTIES OUTPUT_NAME "talker.js")
set_target_properties(listener PROPERTIES OUTPUT_NAME "listener.js")
set_target_properties(timers PROPERTIES OUTPUT_NAME "timers.js")
set_target_properties(bool_client PROPERTIES OUTPUT_NAME "bool_client.js")
endif()

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(talker ${catkin_EXPORTED_TARGETS})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})
add_dependencies(timers ${catkin_EXPORTED_TARGETS})
add_dependencies(bool_client ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if (NOT ${ROSWASM_NATIVE})
target_link_libraries(talker ${roswasm_LIBRARIES})
target_link_libraries(listener ${roswasm_LIBRARIES})
target_link_libraries(timers ${roswasm_LIBRARIES})
target_link_libraries(bool_client ${roswasm_LIBRARIES})
else()
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(timers ${catkin_LIBRARIES})
target_link_libraries(bool_client ${catkin_LIBRARIES})
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
if (NOT ${ROSWASM_NATIVE})
install(PROGRAMS www/timers.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/timers.wasm www/listener.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/listener.wasm www/talker.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/talker.wasm www/bool_client.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/bool_client.wasm
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS talker listener timers bool_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/bool_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Testing only works for melodic right now
if ($ENV{ROS_DISTRO} STREQUAL "melodic")
  if(NOT ${ROSWASM_NATIVE} AND CATKIN_ENABLE_TESTING)
    find_package(rostest)
    add_rostest(test/test-talker-listener.launch)
  endif()
endif()

else() # no emsdk

find_package(catkin REQUIRED COMPONENTS roscpp roswasm std_msgs)
catkin_package()

configure_file(www/talker.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/talker.html COPYONLY)
configure_file(www/listener.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/listener.html COPYONLY)
configure_file(www/timers.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/timers.html COPYONLY)
configure_file(www/bool_client.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/bool_client.html COPYONLY)

endif()
