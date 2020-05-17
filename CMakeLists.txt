#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0015 NEW)

project(roswasm)

set(CMAKE_C_COMPILER emcc)
set(CMAKE_CXX_COMPILER em++)

#set(EMSDK_ROOT $ENV{EMSDK})
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${EMSDK_ROOT}/upstream/emscripten/cmake/Modules")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${EMSDK_ROOT}/upstream/emscripten/cmake/Modules")
#set(CMAKE_TOOLCHAIN_FILE "${CMAKE_MODULE_PATH}/Platform/Emscripten.cmake")

#find_package(Emscripten REQUIRED)

message("EMscripten include dirs: ${Emscripten_INCLUDE_DIRS}")
message("Cmake toolchain: ${CMAKE_TOOLCHAIN_FILE}")
message("Emscripten dir: ${EMSDK_ROOT}")

# setting EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON here does not
# work, since it will only be taken into account on the next cmake
# configuration. So we explicitly run the code guarded by
# EMSCRIPTEN_GENERATE_BITCODE_STATIC_LIBRARIES=ON in Emscripten.cmake
# platform file.
SET(CMAKE_STATIC_LIBRARY_SUFFIX ".bc")
SET(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_C_COMPILER> -o <TARGET> <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_CXX_COMPILER> -o <TARGET> <LINK_FLAGS> <OBJECTS>")
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
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build.emscripten)

#include_directories("lib/assimp/include")

#link_directories("lib/assimp/lib-js")
#link_libraries("assimp")

include_directories(
    include
    /opt/ros/melodic/include
    external/cbor-lite/include
    external/roscpp_json_serialize/include
    external/roscpp_json_serialize/external/rapidjson/include
)

add_library(roswasm src/roswasm.cpp)

add_executable(roswasm_test src/test.cpp)
set_target_properties(roswasm_test PROPERTIES OUTPUT_NAME "test.js")

target_link_libraries(roswasm -lwebsocket.js)
target_link_libraries(roswasm_test roswasm -lwebsocket.js)

#set_target_properties(roswasm_test PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")
#set_target_properties(roswasm PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 --preload-file ${CMAKE_SOURCE_DIR}/assets --bind")

