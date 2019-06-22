cmake_minimum_required(VERSION 2.8)

project(AUV_TASKS)

find_package( OpenCV 3.2 REQUIRED )

include_directories(include/)

add_library(GateEdge include/GateEdge.h src/GateEdge.cpp)
target_link_libraries( GateEdge ${OpenCV_LIBS} )

add_executable(gatetest src/test.cpp)
target_link_libraries(gatetest ${OpenCV_LIBS})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${OpenCV_LIBS})
