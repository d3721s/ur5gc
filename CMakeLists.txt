cmake_minimum_required(VERSION 3.5)

project(ur5gc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_COMPILER gcc-10)
set(CMAKE_CXX_COMPILER g++-10)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

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
add_compile_options(-Wno-ignored-attributes)
add_executable(ur5gc main.cpp ur5.cpp)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
target_link_libraries(
    ur5gc
    libpinocchio.so
    libhpp-fcl.so
#    liburcl.so
    librtde.so
    pthread
    ${Boost_LIBRARIES}
)
