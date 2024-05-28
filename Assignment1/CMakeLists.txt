cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer main.cpp rasterizer.hpp Triangle.hpp Triangle.cpp rasterizer.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Rasterizer PRIVATE Eigen3::Eigen)