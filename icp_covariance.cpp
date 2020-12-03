#include "icp_covariance.h"

#include <iostream>

int main(int argc, char *argv[]) {
  std::cout << "icp_covariance test" << std::endl;

  // step 1: generate data
  std::vector<Eigen::Vector3d> pcl1;
  std::vector<Eigen::Vector3d> pcl2;
  Eigen::Affine3d init_pose;
  icp_cov::data_gen::DataGeneration(pcl1, pcl2, init_pose);

  // step 2: icp

  // step 3: calc icp covariance
  // true / hessian / proposed

  // step 4: visualization

  return 0;
}
