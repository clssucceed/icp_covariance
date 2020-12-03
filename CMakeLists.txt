project(icp_covariance)
find_package(Eigen3)


add_executable(icp_covariance icp_covariance.cpp data_generation.cpp utils.cpp)
target_link_libraries(icp_covariance Eigen3::Eigen)