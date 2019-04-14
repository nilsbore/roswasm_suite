enable_testing()

add_executable(unit main.cpp basic.cpp codec.cpp canon.cpp exception.cpp)

target_compile_definitions(unit PRIVATE "BOOST_TEST_DYN_LINK=1")
target_link_libraries(unit ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_custom_target(unit-tests
    COMMENT Unit Tests
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/unit --log_level=warning
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_custom_target(junit
    COMMENT Unit Tests with JUNIT-format results output
    COMMAND mkdir -p ${PROJECT_BINARY_DIR}/test-reports
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/unit --log_level=warning -f JUNIT -k ${PROJECT_BINARY_DIR}/test-reports/unit.xml
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
