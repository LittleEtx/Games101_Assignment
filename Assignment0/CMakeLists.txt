cmake_minimum_required(VERSION 3.12)
project(Transformation)

add_executable(Transformation1 homework.cpp)
add_executable(Transformation2 main.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(Transformation1 PRIVATE Eigen3::Eigen)
target_link_libraries(Transformation2 PRIVATE Eigen3::Eigen)


