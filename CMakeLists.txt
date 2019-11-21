cmake_minimum_required(VERSION 3.15)

set(PROJECT_VER_MAJOR 0)
set(PROJECT_VER_MINOR 1)
set(PROJECT_VER_PATCH 0)
set(PROJECT_VER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_CXX_FLAGS "-std=c++11")
Message("Build type: " ${CMAKE_BUILD_TYPE})

project(SimpleV)

add_subdirectory(src)
add_subdirectory(test)
# aux_source_directory(src SRC_DIR)
set(SRC "${CMAKE_PROJECT_DIR}/src")
set(TEST "${CMAKE_PROJECT_DIR}/test")
set(THIRDPARTY "${CMAKE_PROJECT_DIR}/thirdparty")
set(ORB_SLAM "${THIRDPARTY}/ORB_SLAM")

# configure_file(config.h.in config.h)
# target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR})

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}
                            TestLib
                            ${OpenCV_LIBS}
                            ${freenect2_LIBRARIES}
                            ${PCL_LIBRARIES})

