cmake_minimum_required(VERSION 3.14)
project(golf VERSION 4.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Version information
set(golf_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(GOLF_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(GOLF_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(GOLF_VERSION ${PROJECT_VERSION})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(PROJECT_TEST_DIR ${PROJECT_SOURCE_DIR}/test)

# Configure version header
configure_file(
    "${PROJECT_INCLUDE_DIR}/version.hpp.in"
    "${PROJECT_BINARY_DIR}/include/version.hpp"
    @ONLY
)

include_directories(${PROJECT_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR}/include)

set(SOURCES
   ${PROJECT_SRC_DIR}/math_utils.cpp
   ${PROJECT_SRC_DIR}/ShotPhysicsContext.cpp
   ${PROJECT_SRC_DIR}/FlightPhase.cpp
   ${PROJECT_SRC_DIR}/FlightSimulator.cpp
   ${PROJECT_SRC_DIR}/ground_physics.cpp
   ${PROJECT_SRC_DIR}/terrain_interface.cpp
)

set(HEADERS
   ${PROJECT_INCLUDE_DIR}/libgolf.hpp
   ${PROJECT_INCLUDE_DIR}/launch_data.hpp
   ${PROJECT_INCLUDE_DIR}/math_utils.hpp
   ${PROJECT_INCLUDE_DIR}/ShotPhysicsContext.hpp
   ${PROJECT_INCLUDE_DIR}/atmospheric_data.hpp
   ${PROJECT_INCLUDE_DIR}/FlightPhase.hpp
   ${PROJECT_INCLUDE_DIR}/FlightSimulator.hpp
   ${PROJECT_INCLUDE_DIR}/BallState.hpp
   ${PROJECT_INCLUDE_DIR}/ground_surface.hpp
   ${PROJECT_INCLUDE_DIR}/GroundProvider.hpp
   ${PROJECT_INCLUDE_DIR}/terrain_interface.hpp
   ${PROJECT_INCLUDE_DIR}/ground_physics.hpp
   ${PROJECT_INCLUDE_DIR}/physics_constants.hpp
)

# Create the static library
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})

# Set library version properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Set compilation flags based on compiler
if(MSVC)
    # MSVC-specific flags
    set(CMAKE_CXX_FLAGS "/W4")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
else()
    # GCC/Clang flags
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math")
endif()

# Code coverage option
option(ENABLE_COVERAGE "Enable code coverage" OFF)
if(ENABLE_COVERAGE AND NOT MSVC)
    message(STATUS "Building with code coverage enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

if(NOT EMSCRIPTEN)
    # Include Google Test
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.17.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Enable testing
    enable_testing()

    # Specify the test source files
    set(TEST_SOURCES
        ${PROJECT_TEST_DIR}/test_aerodynamic_model.cpp
        ${PROJECT_TEST_DIR}/test_ball_physics_vars.cpp
        ${PROJECT_TEST_DIR}/test_math_utils.cpp
        ${PROJECT_TEST_DIR}/test_bounce_phase.cpp
        ${PROJECT_TEST_DIR}/test_roll_phase.cpp
        ${PROJECT_TEST_DIR}/test_flight_simulator.cpp
        ${PROJECT_TEST_DIR}/test_ground_provider.cpp
        ${PROJECT_TEST_DIR}/test_terrain_interface.cpp
        ${PROJECT_TEST_DIR}/test_ground_physics.cpp
        ${PROJECT_SRC_DIR}/ShotPhysicsContext.cpp
        ${PROJECT_SRC_DIR}/math_utils.cpp
    )

    # Add test executable and link with gtest_main and the static library
    add_executable(libgolf_tests ${TEST_SOURCES})
    target_link_libraries(libgolf_tests ${PROJECT_NAME} GTest::gtest_main)

    # Examples
    add_executable(calculate_ball_landing examples/calculate_ball_landing.cpp)
    target_link_libraries(calculate_ball_landing ${PROJECT_NAME})

    add_executable(calculate_ball_trajectory examples/calculate_ball_trajectory.cpp)
    target_link_libraries(calculate_ball_trajectory ${PROJECT_NAME})

    add_executable(multi_ground_simulation examples/multi_ground_simulation.cpp)
    target_link_libraries(multi_ground_simulation ${PROJECT_NAME})

    # Discover tests
    include(GoogleTest)
    gtest_discover_tests(libgolf_tests)
endif()

if(EMSCRIPTEN)
    # JavaScript/wasm binding for the web visualizer
    add_executable(libgolf_wasm wasm/bindings.cpp)
    target_link_libraries(libgolf_wasm PRIVATE ${PROJECT_NAME})
    set_target_properties(libgolf_wasm PROPERTIES SUFFIX ".js")
    target_link_options(libgolf_wasm PRIVATE
        "-lembind"
        "-sEXPORT_ES6=1"
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createLibGolf"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sENVIRONMENT=web"
    )
endif()

install(TARGETS golf DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${PROJECT_BINARY_DIR}/include/version.hpp" DESTINATION include)