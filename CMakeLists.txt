cmake_minimum_required(VERSION 3.15)
project(sm64_bully_movement
  DESCRIPTION "Bruteforces bully speed/angle combinations for triple-bully-collision displacement"
)

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