cmake_minimum_required(VERSION 3.16)

project(geminiastro_indi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_INDI_DRIVER "Build the INDI driver executable" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_library(geminiastro_protocol
    src/geminiastro_protocol.cpp
)
target_include_directories(geminiastro_protocol PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if(UNIX)
    add_executable(geminiastro_probe
        src/geminiastro_probe.cpp
    )
    target_link_libraries(geminiastro_probe PRIVATE geminiastro_protocol)
endif()

if(BUILD_INDI_DRIVER)
    find_package(INDI QUIET)

    if(NOT INDI_FOUND)
        message(FATAL_ERROR "INDI development files were not found. Install libindi-dev or configure with -DBUILD_INDI_DRIVER=OFF.")
    endif()

    add_executable(indi_geminiastro_focus
        src/geminiastro_focuser.cpp
    )
    target_include_directories(indi_geminiastro_focus PRIVATE ${INDI_INCLUDE_DIRS})
    target_link_libraries(indi_geminiastro_focus PRIVATE geminiastro_protocol ${INDI_LIBRARIES})

    install(TARGETS indi_geminiastro_focus RUNTIME DESTINATION bin)
    install(FILES indi_geminiastro_focus.xml DESTINATION share/indi)
endif()

enable_testing()

add_executable(test_protocol
    tests/test_protocol.cpp
)
target_link_libraries(test_protocol PRIVATE geminiastro_protocol)
add_test(NAME protocol COMMAND test_protocol)
