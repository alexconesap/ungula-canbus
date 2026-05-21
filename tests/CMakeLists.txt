cmake_minimum_required(VERSION 3.16)
project(ungula_canbus_lib_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# UngulaCanbus sits on top of UngulaHal; both source trees are
# sibling libraries in the workspace.
set(LIB_CANBUS_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)
set(LIB_HAL_SRC    ${CMAKE_CURRENT_LIST_DIR}/../../lib_hal/src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${LIB_CANBUS_SRC}
    ${LIB_HAL_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/ArduinoStub
)

# Host build: lib_hal's `can_default.cpp` provides a no-op `Can` that
# satisfies `ICan`. Real tests use a `FakeCan` (added in step 3) for
# anything that needs traffic.
add_executable(test_ungula_canbus
    test_canbus_smoke.cpp
    test_bus_helpers.cpp
    test_rmd_protocol.cpp
    test_rmd_motor.cpp
    ${LIB_CANBUS_SRC}/ungula/canbus/bus_helpers.cpp
    ${LIB_CANBUS_SRC}/ungula/canbus/devices/rmd/motor.cpp
    ${LIB_HAL_SRC}/ungula/hal/can/platforms/can_default.cpp
)
target_link_libraries(test_ungula_canbus gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_ungula_canbus)
