project(realsense-testing)
cmake_minimum_required(VERSION 2.8)

add_executable (pcl_testing pcl_testing.cpp)
#add_executable (pcl_testing_2 pcl_testing_2.cpp)
add_executable (pcl_registration pcl_registration.cpp)

find_package(PCL 1.6 REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
#find_package(OpenGL REQUIRED)
#find_package(glfw3 3.2 REQUIRED)

#target_include_directories(pcl_testing_2 ${OPENGL_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries (pcl_testing ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS})
#target_link_libraries (pcl_testing_2 ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS})
target_link_libraries (pcl_registration ${PCL_LIBRARIES})
