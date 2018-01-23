cmake_minimum_required (VERSION 2.8.11)
project (cbor-lite)

set(CMAKE_CXX_STANDARD 14)

add_library(cbor-lite INTERFACE)
target_include_directories(cbor-lite INTERFACE include/)

file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)
add_custom_target(reformat COMMAND clang-format -i ${ALL_SOURCE_FILES})

add_custom_target(junit COMMAND mkdir -p test-results && ./unit/unit -f JUNIT -k test-results/results.xml)

add_subdirectory (unit)
