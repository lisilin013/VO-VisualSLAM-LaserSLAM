cmake_minimum_required(VERSION 3.5)
project(hw1)

set(CMAKE_CXX_STANDARD 14)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(update_RQ main.cpp)
target_link_libraries(update_RQ ${Sophus_LIBRARIES})
