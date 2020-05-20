cmake_minimum_required(VERSION 2.8.3)
project(roswasm)

if (DEFINED ENV{EMSDK})

set(CMAKE_C_COMPILER emcc)
set(CMAKE_CXX_COMPILER em++)

#set(EMSDK_ROOT $ENV{EMSDK})
find_package(catkin REQUIRED COMPONENTS roscpp roscpp_json_serialize std_msgs sensor_msgs geometry_msgs nav_msgs) # rosmon_msgs)

catkin_package(
  INCLUDE_DIRS include external/cbor-lite/include
  LIBRARIES roswasm
  CATKIN_DEPENDS roscpp roscpp_json_serialize
#  DEPENDS system_lib
)

message("EMscripten include dirs: ${Emscripten_INCLUDE_DIRS}")
message("Cmake toolchain: ${CMAKE_TOOLCHAIN_FILE}")
message("Emscripten dir: ${EMSDK_ROOT}")

#unset(ROSCONSOLE_BACKEND_LOG4CXX)
#unset(ROS_BUILD_SHARED_LIBS)
#set(ROS_BUILD_SHARED_LIBS 0)
message("FLAGS: ${CMAKE_CXX_FLAGS}")

# setting EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON here does not
# work, since it will only be taken into account on the next cmake
# configuration. So we explicitly run the code guarded by
# EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON in Emscripten.cmake
# platform file.
set(CMAKE_STATIC_LIBRARY_SUFFIX ".bc")
set(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_C_COMPILER> -o <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_CXX_COMPILER> -o <TARGET> <LINK_FLAGS> <OBJECTS>")
# --- end EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES

set(EMCC_FLAGS "")
#set(EMCC_FLAGS "${EMCC_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
set(EMCC_FLAGS "${EMCC_FLAGS} -s ERROR_ON_UNDEFINED_SYMBOLS=1")
set(EMCC_FLAGS "${EMCC_FLAGS} -s ALLOW_MEMORY_GROWTH=1")
#set(EMCC_FLAGS "${EMCC_FLAGS} -s ASSERTIONS=1")
set(EMCC_FLAGS "${EMCC_FLAGS} -s WASM=1")
set(EMCC_FLAGS "${EMCC_FLAGS} -Wno-warn-absolute-paths")
#set(EMCC_FLAGS "${EMCC_FLAGS} --memory-init-file 0")
set(EMCC_FLAGS "${EMCC_FLAGS} -s USE_BOOST_HEADERS=1")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMCC_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EMCC_FLAGS}")

add_definitions("-g4 -O2 -std=c++11") # -lwebsocket.js -O2 -s WASM=1 -s USE_BOOST_HEADERS=1")

#set(CMAKE_VERBOSE_MAKEFILE on)

include_directories(
  include
  #/usr/include
  #/opt/ros/melodic/include
  external/cbor-lite/include
  #external/roscpp_json_serialize/include
  #external/roscpp_json_serialize/external/rapidjson/include
  ${catkin_INCLUDE_DIRS}
)

add_library(roswasm STATIC src/roswasm.cpp)
#set_target_properties(roswasm PROPERTIES SUFFIX ".bc")

add_executable(roswasm_test src/test.cpp)
set_target_properties(roswasm_test PROPERTIES OUTPUT_NAME "test.js")

target_link_libraries(roswasm -lwebsocket.js)
target_link_libraries(roswasm_test roswasm -lwebsocket.js)

configure_file(www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.html COPYONLY)

#set_target_properties(roswasm_test PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")
#set_target_properties(roswasm PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.wasm scripts/run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS roswasm_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS roswasm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

else() # no emsdk

find_package(catkin REQUIRED COMPONENTS roscpp roscpp_json_serialize std_msgs sensor_msgs geometry_msgs nav_msgs) # rosmon_msgs)
catkin_package()

configure_file(www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.html COPYONLY)
configure_file(www/test.wasm ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.wasm COPYONLY)
configure_file(www/test.js ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.js COPYONLY)

endif()
