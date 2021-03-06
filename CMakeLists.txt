cmake_minimum_required(VERSION 3.10)
project(icp_covariance)
add_definitions(-std=c++11)
find_package(Eigen3)
find_package(OpenCV)
find_package(PCL)
find_package(yaml-cpp REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
message("${OpenCV_INCLUDE_DIRS}")
message("pcl include dirs: ${PCL_INCLUDE_DIRS}")
message("pcl library dirs: ${PCL_LIBRARY_DIRS}")
add_executable(icp_covariance 
  app.cpp 
  data_generator.cpp 
  image_data_generator.cpp
  finite_rectangle.cpp
  utils.cpp 
  visualization.cpp
  pcl_alignment.cpp
  icp_covariance.cpp
  config/config.cpp)

target_link_libraries(icp_covariance Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp)