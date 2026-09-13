cmake_minimum_required(VERSION 3.16)
project(MotorControlSim
    VERSION   1.0.0
    LANGUAGES CXX
    DESCRIPTION "Closed-loop PID motor-control simulation"
)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Include paths ─────────────────────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ── Core library (shared by simulation and tests) ────────────────────────────
add_library(motor_core STATIC
    src/Motor.cpp
    src/Encoder.cpp
    src/PIDController.cpp
    src/SafetySystem.cpp
    src/MotorController.cpp
)

# ── Simulation executable ─────────────────────────────────────────────────────
add_executable(motor_sim src/main.cpp)
target_link_libraries(motor_sim PRIVATE motor_core)

# ── Test executable ───────────────────────────────────────────────────────────
add_executable(motor_tests
    tests/test_main.cpp
    tests/test_pid.cpp
    tests/test_safety.cpp
)
target_include_directories(motor_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests
)
target_link_libraries(motor_tests PRIVATE motor_core)

# ── CTest integration ────────────────────────────────────────────────────────
enable_testing()
add_test(NAME unit_tests COMMAND motor_tests)
