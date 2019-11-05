include_directories(../include)
find_package(OpenCV REQUIRED)

aux_source_directory(. DIR_LIB_SRC)

add_library(TestLib ${DIR_LIB_SRC})
target_link_libraries(TestLib ${OpenCV_LIBS})

