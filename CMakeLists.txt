cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(ShowCloudPoints)

set(CMAKE_CXX_STANDARD 14)
#set( CMAKE_VERBOSE_MAKEFILE on )

set(PCL_DIR /usr/local/Cellar/pcl/1.9.1_4/share/pcl-1.9)
find_package(PCL 1.4 REQUIRED)

#find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
#link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${LIBLAS_DEFINITIONS})

add_executable(ShowCloudPoints iss_kp.cpp)
#target_link_libraries (ShowCloudPoints ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (ShowCloudPoints ${PCL_LIBRARIES})
