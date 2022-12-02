cmake_minimum_required(VERSION 3.22)
project(TNP-TP3)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS       "-Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g3")

include_directories(eigen src)

add_executable(main
    src/main.cpp
    src/kdtree.cpp
    src/obj.cpp
    src/ransac.cpp)
