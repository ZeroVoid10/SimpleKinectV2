cmake_minimum_required(VERSION 3.15)

set(PROJECT_VER_MAJOR 0)
set(PROJECT_VER_MINOR 1)
set(PROJECT_VER_PATCH 0)
set(PROJECT_VER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")
set(CMAKE_BUILD_TYPE "Debug")

project(RugbyDetect)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(RugbyDetect test.cpp)
add_executable(CloudPointTest cpTest.cpp)

target_link_libraries(RugbyDetect ${OpenCV_LIBS})
target_link_libraries(RugbyDetect ${freenect2_LIBRARIES})
target_link_libraries(RugbyDetect ${PCL_LIBRARIES})
target_link_libraries(CloudPointTest ${OpenCV_LIBS})
target_link_libraries(CloudPointTest ${freenect2_LIBRARIES})
target_link_libraries(CloudPointTest ${PCL_LIBRARIES})

