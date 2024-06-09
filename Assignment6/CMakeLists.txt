cmake_minimum_required(VERSION 3.10)
project(RayTracing1)

set(CMAKE_CXX_STANDARD 17)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

find_package(OpenCV REQUIRED)

add_executable(RayTracing1 main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

target_link_libraries(RayTracing1 ${OpenCV_LIBS})
