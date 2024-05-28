cmake_minimum_required(VERSION 3.12)
project(Transformation)

add_executable(Transformation homework.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)


