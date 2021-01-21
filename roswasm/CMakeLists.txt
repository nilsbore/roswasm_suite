cmake_minimum_required(VERSION 2.8.3)
project(roswasm)

option(ROSWASM_NATIVE "Compile with ROS Native version of ROSWASM" OFF)

if (DEFINED ENV{EMSDK} OR ${ROSWASM_NATIVE})

#set(EMSDK_ROOT $ENV{EMSDK})
find_package(catkin REQUIRED COMPONENTS roscpp roscpp_json_serialize std_msgs sensor_msgs geometry_msgs nav_msgs) # rosmon_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if (NOT ${ROSWASM_NATIVE})
set(ROSWASM_LIB roswasm)
catkin_package(
  INCLUDE_DIRS include external/cbor-lite/include
  LIBRARIES #roswasm
  CFG_EXTRAS roswasm-extras.cmake
  CATKIN_DEPENDS roscpp_json_serialize
#  DEPENDS system_lib
)
else()
set(ROSWASM_LIB roswasm_native)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${ROSWASM_LIB}
  CFG_EXTRAS roswasm-extras.cmake
#  CATKIN_DEPENDS roscpp_json_serialize
#  DEPENDS system_lib
)
endif()

###########
## Build ##
###########

if (NOT ${ROSWASM_NATIVE})

set(CMAKE_C_COMPILER "$ENV{EMSDK}/upstream/emscripten/emcc")
set(CMAKE_CXX_COMPILER "$ENV{EMSDK}/upstream/emscripten/em++")

# setting EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON here does not
# work, since it will only be taken into account on the next cmake
# configuration. So we explicitly run the code guarded by
# EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON in Emscripten.cmake
# platform file.
#set(CMAKE_STATIC_LIBRARY_SUFFIX ".bc")
#set(CMAKE_STATIC_LIBRARY_SUFFIX ".wasm")
#set(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_C_COMPILER> -r -o <TARGET> <LINK_FLAGS> <OBJECTS>")
#set(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_CXX_COMPILER> -r -o <TARGET> <LINK_FLAGS> <OBJECTS>")
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
set(EMCC_FLAGS "${EMCC_FLAGS} -fno-stack-protector")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMCC_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EMCC_FLAGS}")

add_definitions("-g4 -O2 -std=c++11") # -lwebsocket.js -O2 -s WASM=1 -s USE_BOOST_HEADERS=1")

else()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROSWASM_NATIVE")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DROSWASM_NATIVE")

add_definitions("-g -O2 -std=c++11") # -lwebsocket.js -O2 -s WASM=1 -s USE_BOOST_HEADERS=1")

endif() # NOT ROSWASM_NATIVE


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

#add_library(roswasm STATIC src/roswasm.cpp)
add_library(${ROSWASM_LIB} src/roswasm.cpp)
#set_target_properties(roswasm PROPERTIES OUTPUT_NAME "libroswasm")

add_executable(roswasm_test src/test.cpp)

if (NOT ${ROSWASM_NATIVE})
set_target_properties(roswasm_test PROPERTIES OUTPUT_NAME "test.js")

target_link_libraries(${ROSWASM_LIB} -lwebsocket.js)
target_link_libraries(roswasm_test ${ROSWASM_LIB} -lwebsocket.js)

configure_file(www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.html COPYONLY)
else()
target_link_libraries(${ROSWASM_LIB} ${catkin_LIBRARIES})
target_link_libraries(roswasm_test ${ROSWASM_LIB} ${catkin_LIBRARIES})
endif()

#set_target_properties(roswasm_test PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")
#set_target_properties(roswasm PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
if (NOT ${ROSWASM_NATIVE})
install(PROGRAMS www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.wasm scripts/run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS roswasm_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${ROSWASM_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/roswasm/ external/cbor-lite/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

else() # no emsdk

find_package(catkin REQUIRED COMPONENTS roscpp roscpp_json_serialize std_msgs sensor_msgs geometry_msgs nav_msgs) # rosmon_msgs)
catkin_package()

configure_file(www/test.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.html COPYONLY)
configure_file(www/test.wasm ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.wasm COPYONLY)
configure_file(www/test.js ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/test.js COPYONLY)

endif()
