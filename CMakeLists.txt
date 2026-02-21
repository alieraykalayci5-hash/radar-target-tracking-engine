cmake_minimum_required(VERSION 3.16)
project(radar_target_tracking_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(radar_tracker
  src/main.cpp
  src/rng.h
  src/fnv1a.h
  src/math_types.h
  src/kalman.h
  src/kalman.cpp
  src/tracker.h
  src/tracker.cpp
  src/sim.h
  src/sim.cpp
  src/csv.h
  src/hungarian.h
  src/hungarian.cpp
)

# Eigen3 (header-only)
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(radar_tracker PRIVATE Eigen3::Eigen)

if (MSVC)
  target_compile_options(radar_tracker PRIVATE /W4 /permissive-)
else()
  target_compile_options(radar_tracker PRIVATE -Wall -Wextra -Wpedantic)
endif()