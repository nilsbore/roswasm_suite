enable_testing()

add_executable(unit unit-main.cpp basic.cpp canon.cpp exception.cpp)

target_compile_definitions(unit PRIVATE "BOOST_TEST_DYN_LINK=1")
target_link_libraries(unit ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
