#include "data_generator.h"
#include "visualization.h"
#include "color.h"
#include <iostream>

int main(int argc, char *argv[]) {
  std::cout << "icp_covariance test" << std::endl;

  // step 1: generate data
  auto data_generator = icp_cov::DataGenerator::Instance();
  std::vector<Eigen::Vector3d> pcl1_in_ego_frame =
      data_generator->pcl1_in_ego_frame();
  std::vector<Eigen::Vector3d> pcl2_in_ego_frame =
      data_generator->pcl2_in_ego_frame();
  auto visualization = icp_cov::Visualization::Instance();
  visualization->DrawPoints(pcl1_in_ego_frame, icp_cov::kColorRed);
  visualization->DrawPoints(pcl2_in_ego_frame, icp_cov::kColorGreen);
  visualization->Show();
  // Eigen::Affine3d init_pose;
  // icp_cov::data_gen::DataGeneration(pcl1, pcl2, init_pose);

  // step 2: icp

  // step 3: calc icp covariance
  // true / hessian / proposed

  // step 4: visualization

  return 0;
}
