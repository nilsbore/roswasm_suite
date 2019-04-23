cmake_minimum_required (VERSION 3.9.5 FATAL_ERROR)
project (cbor-lite)

if(POLICY CMP0076)
  cmake_policy(SET CMP0076 OLD)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "C++ compiler is ${CMAKE_CXX_COMPILER_ID}")

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(WARNING "Not a 64-bit system! YMMV")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/permissive-)
    add_definitions(/D__ORDER_LITTLE_ENDIAN__=1 /D__ORDER_BIG_ENDIAN__=0 /D__BYTE_ORDER__=1)
else()
    add_compile_options(-O0)
    add_compile_options(-Wall -Wextra -pedantic -Wold-style-cast)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        add_compile_options(-Wno-implicit-fallthrough)
    endif()
endif()

add_definitions(-DBOOST_NO_AUTO_PTR)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/include Boost::boost)

add_library(cbor-lite INTERFACE)
target_include_directories(cbor-lite INTERFACE include/)

add_subdirectory (unit)

file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)
add_custom_target(reformat COMMAND clang-format -i ${ALL_SOURCE_FILES})

