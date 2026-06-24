cmake_minimum_required(VERSION 3.16)
project(StewartSim_V3 VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen path
if(WIN32)
    set(EIGEN_INCLUDE C:/libs/eigen)
else()
    set(EIGEN_INCLUDE /usr/include/eigen3)
endif()

# Shared sources — no main.cpp
set(LIB_SOURCES
    actuator/MotorDriver.cpp
    actuator/ActuatorState.cpp
    geometry/Geometry.cpp
    geometry/Kinematics.cpp
    platform/StewartPlatform.cpp
    platform/SafetyMonitor.cpp
    threading/ThreadedFOCDriver.cpp
    threading/ThreadedController.cpp
    threading/ThreadedSafetyMonitor.cpp
    threading/Logger.cpp
    control/TaskSpaceController.cpp
)

# Main executable
add_executable(StewartSim_V3 main.cpp ${LIB_SOURCES})
target_include_directories(StewartSim_V3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN_INCLUDE})

if(UNIX)
    target_link_libraries(StewartSim_V3 PRIVATE pthread m)
endif()

# gtest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)
enable_testing()

# Test executable
add_executable(stewart_tests
    tests/test_kinematics.cpp
    tests/test_controllers.cpp
    tests/test_utils.cpp
    ${LIB_SOURCES}
)

target_include_directories(stewart_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN_INCLUDE})

target_link_libraries(stewart_tests GTest::gtest_main)

if(UNIX)
    target_link_libraries(stewart_tests PRIVATE pthread m)
endif()

include(GoogleTest)
gtest_discover_tests(stewart_tests)