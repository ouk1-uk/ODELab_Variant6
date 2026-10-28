cmake_minimum_required(VERSION 3.21)
project(ODELabVariant6 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ODELib STATIC src/ODETask.cpp)
target_include_directories(ODELib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(ODEDemo demo/main.cpp)
target_link_libraries(ODEDemo PRIVATE ODELib)

add_executable(AntipatternDemo antipattern/main.cpp)

# Default: full student project, using Google Test installed by vcpkg.
# Local smoke verification without gtest: -DODELAB_BUILD_TESTS=OFF.
option(ODELAB_BUILD_TESTS "Build Google Test suite" ON)
if(ODELAB_BUILD_TESTS)
    find_package(GTest CONFIG REQUIRED)
    enable_testing()
    add_executable(ODETests tests/ODETests.cpp)
    target_link_libraries(ODETests PRIVATE ODELib GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(ODETests)
endif()
