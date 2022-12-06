cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

set(OpenMP_C "${CMAKE_C_COMPILER}")
set(OpenMP_C_FLAGS "-Xclang -fopenmp -I/usr/local/opt/libomp/include")
set(OpenMP_C_LIB_NAMES "libomp")
set(OpenMP_libomp_LIBRARY "omp")

set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
set(OpenMP_CXX_FLAGS "-Xclang -fopenmp -I/usr/local/opt/libomp/include")
set(OpenMP_CXX_LIB_NAMES "libomp")
set(OpenMP_libomp_LIBRARY "omp")

link_directories("/usr/local/opt/libomp/lib")

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

find_package(OpenMP)
if (OpenMP_CXX_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

