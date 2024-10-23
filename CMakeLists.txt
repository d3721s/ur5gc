cmake_minimum_required(VERSION 3.5)

project(ur5gc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_COMPILER gcc-10)
set(CMAKE_CXX_COMPILER g++-10)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include
    /usr/include
    /opt/ros/noetic/include
    /usr/include/eigen3
    /opt/ros/noetic/lib/x86_64-linux-gnu

)
link_directories(
    /opt/ros/noetic/lib
    /opt/ros/noetic/lib/x86_64-linux-gnu
)
add_executable(ur5gc main.cpp)


target_link_libraries(
    ur5gc
    libpinocchio.so
    libhpp-fcl.so
)
