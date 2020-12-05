#include <iostream>

#include "color.h"
#include "data_generator.h"
#include "utils.h"
#include "visualization.h"

int main(int argc, char *argv[]) {
  std::cout << "icp_covariance test" << std::endl;

  // step 1: generate data
  // pcl
  auto data_generator = icp_cov::DataGenerator::Instance();
  auto pcl1_in_ego_frame = data_generator->pcl1_in_ego_frame();
  auto pcl2_in_ego_frame = data_generator->pcl2_in_ego_frame();
  auto pcl1_in_ego_frame_with_noise =
      data_generator->pcl1_in_ego_frame_with_noise();
  auto pcl2_in_ego_frame_with_noise =
      data_generator->pcl2_in_ego_frame_with_noise();
  auto visualization = icp_cov::Visualization::Instance();
  visualization->DrawPoints(pcl1_in_ego_frame, icp_cov::kColorRed);
  visualization->DrawPoints(pcl2_in_ego_frame, icp_cov::kColorGreen);
  visualization->DrawPoints(pcl1_in_ego_frame_with_noise, icp_cov::kColorPink);
  visualization->DrawPoints(pcl2_in_ego_frame_with_noise, icp_cov::kColorCyan);
  visualization->Show();
  // icp transform
  Eigen::Affine3d icp_transform_gt = data_generator->icp_transform();
  Eigen::Affine3d icp_transform_init = data_generator->icp_transform_init();
  std::cout << "icp_transform_gt: "
            << icp_cov::utils::R2ypr(icp_transform_gt.rotation()).transpose()
            << "; " << icp_transform_gt.translation().transpose() << std::endl;
  std::cout << "icp_transform_init: "
            << icp_cov::utils::R2ypr(icp_transform_init.rotation()).transpose()
            << "; " << icp_transform_init.translation().transpose()
            << std::endl;

  // step 2: icp

  // step 3: calc icp covariance
  // true / hessian / proposed

  // step 4: visualization

  return 0;
}
