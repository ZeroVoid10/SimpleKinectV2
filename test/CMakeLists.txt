cmake_minimum_required(VERSION 3.15)

project(Test)

find_package(freenect2 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)
find_package(OpenCV REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} include)
include_directories(../include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(n_test n_test.cpp)
add_executable(KinectTest test.cpp)
add_executable(CloudPointTest cpTest.cpp)
add_executable(Calibrate caliberation_test.cpp)
add_executable(surf_test surf_test.cpp)

# target_link_libraries(n_test ${freenect2_LIBRARIES} TestLib)
target_link_libraries(Calibrate 
                            ${OpenCV_LIBS})                           
target_link_libraries(surf_test
                            ${OpenCV_LIBS})                           
target_link_libraries(KinectTest 
                            ${OpenCV_LIBS}
                            ${freenect2_LIBRARIES}
                            ${PCL_LIBRARIES})
target_link_libraries(CloudPointTest 
                            ${OpenCV_LIBS}
                            ${freenect2_LIBRARIES}
                            ${PCL_LIBRARIES})
