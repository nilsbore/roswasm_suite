cmake_minimum_required (VERSION 2.8.11)
project (cbor-lite)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-O0)
add_compile_options(-Wall -Wextra -pedantic -Wold-style-cast)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_definitions(-DBOOST_NO_AUTO_PTR)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_library(cbor-lite INTERFACE)
target_include_directories(cbor-lite INTERFACE include/)

file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)
add_custom_target(reformat COMMAND clang-format -i ${ALL_SOURCE_FILES})

add_custom_target(unit-test COMMAND ./unit/unit --log_level=warning)
add_custom_target(junit COMMAND mkdir -p test-results && ./unit/unit --logger=JUNIT,warning,test-results/results.xml)

add_subdirectory (unit)
