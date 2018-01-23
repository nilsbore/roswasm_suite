enable_testing()

set(Boost_ADDITIONAL_VERSIONS "1.62" "1.62.0")
find_package(Boost 1.62 REQUIRED COMPONENTS unit_test_framework)

add_executable(unit unit-main.cpp basic.cpp canon.cpp exception.cpp)

target_include_directories(unit PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_definitions(unit PRIVATE "BOOST_TEST_DYN_LINK=1")
target_link_libraries(unit ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

