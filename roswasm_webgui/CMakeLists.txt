cmake_minimum_required(VERSION 2.8.3)
project(roswasm_webgui)

option(ROSWASM_NATIVE "Compile with ROS Native version of ROSWASM" OFF)

if (DEFINED ENV{EMSDK} OR ${ROSWASM_NATIVE})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp roswasm std_msgs sensor_msgs geometry_msgs nav_msgs rosmon_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

#set(CMAKE_FIND_LIBRARY_SUFFIXES "${CMAKE_FIND_LIBRARY_SUFFIXES} .bc")
if (NOT ${ROSWASM_NATIVE})
set(ROSWASM_WEBGUI_LIB roswasm_webgui)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #roswasm_webgui
  CFG_EXTRAS roswasm_webgui-extras.cmake
  CATKIN_DEPENDS roswasm
#  DEPENDS system_lib
)
else()
set(ROSWASM_WEBGUI_LIB roswasm_webgui_native)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${ROSWASM_WEBGUI_LIB}
  CFG_EXTRAS roswasm_webgui-extras.cmake
  CATKIN_DEPENDS roswasm
#  DEPENDS system_lib
)
endif()

###########
## Build ##
###########

if (${ROSWASM_NATIVE})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
#set(OpenGL_GL_PREFERENCE "GLVND")
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(SDL2 REQUIRED) 
find_package(SDL2_image REQUIRED)
else()

set(EMCC_FLAGS "")

set(EMCC_FLAGS "${EMCC_FLAGS} -s USE_WEBGL2=1")
set(EMCC_FLAGS "${EMCC_FLAGS} -s USE_GLFW=3")
set(EMCC_FLAGS "${EMCC_FLAGS} -s FULL_ES3=1")
set(EMCC_FLAGS "${EMCC_FLAGS} -s USE_SDL=2")
set(EMCC_FLAGS "${EMCC_FLAGS} -s USE_SDL_IMAGE=2")
set(EMCC_FLAGS "${EMCC_FLAGS} -s SDL2_IMAGE_FORMATS='[\"png\", \"jpg\"]'")
#set(EMCC_FLAGS "${EMCC_FLAGS} --preload-file data")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMCC_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EMCC_FLAGS}")

endif() # NOT ${ROSWASM_NATIVE}

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (${ROSWASM_NATIVE})
include_directories(
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIR}
  ${SDL2_IMAGE_INCLUDE_DIRS}
)
endif()

add_library(${ROSWASM_WEBGUI_LIB} src/imgui/imgui_impl_glfw.cpp src/imgui/imgui_impl_opengl3.cpp src/imgui/imgui.cpp src/imgui/imgui_draw.cpp src/imgui/imgui_demo.cpp src/roswasm_webgui/roswasm_monlaunch.cpp src/roswasm_webgui/roswasm_image.cpp src/roswasm_webgui/roswasm_widget.cpp src/roswasm_webgui/roswasm_examples.cpp) # src/roswasm_sam.cpp)
#add_library(roswasm_webgui STATIC src/imgui/imgui_impl_glfw.cpp src/imgui/imgui_impl_opengl3.cpp src/imgui/imgui.cpp src/imgui/imgui_draw.cpp src/imgui/imgui_demo.cpp src/roswasm_webgui/roswasm_monlaunch.cpp src/roswasm_webgui/roswasm_image.cpp src/roswasm_webgui/roswasm_widget.cpp src/roswasm_webgui/roswasm_examples.cpp) # src/roswasm_sam.cpp)
add_dependencies(${ROSWASM_WEBGUI_LIB} ${catkin_EXPORTED_TARGETS})
#)
#set_target_properties(roswasm PROPERTIES SUFFIX ".bc")

add_executable(roswasm_webgui_test src/main.cpp)
add_dependencies(roswasm_webgui_test ${catkin_EXPORTED_TARGETS})
if (NOT ${ROSWASM_NATIVE})
set_target_properties(roswasm_webgui_test PROPERTIES OUTPUT_NAME "webgui.js")
endif()

add_executable(monlaunch_test src/monlaunch_test.cpp)
add_dependencies(monlaunch_test ${catkin_EXPORTED_TARGETS})
if (NOT ${ROSWASM_NATIVE})
set_target_properties(monlaunch_test PROPERTIES OUTPUT_NAME "monlaunch.js")
endif()

if (NOT ${ROSWASM_NATIVE})
target_link_libraries(${ROSWASM_WEBGUI_LIB} ${roswasm_LIBRARIES} -lGL)
target_link_libraries(roswasm_webgui_test ${ROSWASM_WEBGUI_LIB} ${roswasm_LIBRARIES} -lGL)
target_link_libraries(monlaunch_test ${ROSWASM_WEBGUI_LIB} ${roswasm_LIBRARIES} -lGL)
else()
target_link_libraries(${ROSWASM_WEBGUI_LIB} ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${SDL2_IMAGE_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARIES} glfw) # -lGL)
target_link_libraries(roswasm_webgui_test ${ROSWASM_WEBGUI_LIB} ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${SDL2_IMAGE_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARIES} glfw) # -lGL)
target_link_libraries(monlaunch_test ${ROSWASM_WEBGUI_LIB} ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${SDL2_IMAGE_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLEW_LIBRARIES} glfw) #  -lGL)
endif()

if (NOT ${ROSWASM_NATIVE})
configure_file(www/webgui.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/webgui.html COPYONLY)
configure_file(www/monlaunch.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/monlaunch.html COPYONLY)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
if (NOT ${ROSWASM_NATIVE})
install(PROGRAMS www/monlaunch.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/monlaunch.wasm www/webgui.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/webgui.wasm scripts/example.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS roswasm_webgui_test monlaunch_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${ROSWASM_WEBGUI_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/roswasm_webgui/ include/imgui/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roswasm_webgui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

else() # no emsdk

find_package(catkin REQUIRED COMPONENTS roscpp roswasm std_msgs sensor_msgs geometry_msgs nav_msgs)
catkin_package()

configure_file(www/webgui.html ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/webgui.html COPYONLY)
configure_file(www/webgui.wasm ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/webgui.wasm COPYONLY)
configure_file(www/webgui.js ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/webgui.js COPYONLY)

endif()
