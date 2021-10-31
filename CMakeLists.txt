cmake_minimum_required(VERSION 3.15)
project(sm64_bully_movement
  LANGUAGES CXX
  DESCRIPTION "Bruteforces bully speed/angle combinations for triple-bully-collision displacement"
)

# Set default build type to Release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(bruteforce
  main.cpp
  Trig.cpp
  Trig.hpp
  SpeedRangeSearch.cpp
  SpeedRangeSearch.hpp
  Constants.hpp
  BullyPath.cpp
  BullyPath.hpp
  Int128.hpp
)

# Equivalent to specifying a bunch of compiler flags.
# ===================================================

# LTO
include(CheckIPOSupported)
check_ipo_supported(RESULT lto_supported OUTPUT lto_error LANGUAGES CXX)
if(lto_supported)
  set_target_properties(bruteforce PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION true
  )
endif()


# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(bruteforce PRIVATE OpenMP::OpenMP_CXX)
endif()

# -march=native with MSVC emulation
include("cmake/march_native.cmake")
set_march_native(bruteforce)

# -O3 is already handled by using release mode.
