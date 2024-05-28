cmake_minimum_required(VERSION 3.10)
project(Rasterizer1)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer1 main.cpp rasterizer.hpp Triangle.hpp Triangle.cpp rasterizer.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Rasterizer1 PRIVATE Eigen3::Eigen)