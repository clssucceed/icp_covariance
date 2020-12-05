cmake_minimum_required(VERSION 3.10)
project(icp_covariance)
find_package(Eigen3)
find_package(OpenCV)

include_directories( ${OpenCV_INCLUDE_DIRS} )
message("${OpenCV_INCLUDE_DIRS}")
add_executable(icp_covariance 
  app.cpp 
  data_generator.cpp 
  utils.cpp 
  visualization.cpp)

target_link_libraries(icp_covariance Eigen3::Eigen ${OpenCV_LIBS})