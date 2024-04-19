cmake_minimum_required(VERSION 3.10)
project(GJK)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS})

add_library(gjk gjk.cpp)
target_link_libraries(gjk PUBLIC Eigen3::Eigen)

# add_executable(my_executable main.cpp)
# target_link_libraries(my_executable PRIVATE gjk)

add_executable(charlie main.cpp)
target_link_libraries(charlie PRIVATE 
    gjk 
    ${OpenCV_LIBRARIES})
