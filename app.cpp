#include <pcl/pcl_config.h>

#include <iostream>

#include "color.h"
#include "data_generator.h"
#include "pcl_alignment.h"
#include "utils.h"
#include "visualization.h"

int main(int argc, char *argv[]) {
  std::cout << PCL_VERSION << std::endl;
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
  auto pcl1_in_world_frame = data_generator->pcl1_in_world_frame();
  auto pcl2_in_world_frame = data_generator->pcl2_in_world_frame();
  auto pcl1_in_world_frame_with_noise =
      data_generator->pcl1_in_world_frame_with_noise();
  auto pcl2_in_world_frame_with_noise =
      data_generator->pcl2_in_world_frame_with_noise();
  auto visualization = icp_cov::Visualization::Instance();
  visualization->DrawPoints(pcl1_in_ego_frame, icp_cov::kColorRed);
  visualization->DrawPoints(pcl2_in_ego_frame, icp_cov::kColorGreen);
  visualization->DrawPoints(pcl1_in_ego_frame_with_noise, icp_cov::kColorPink);
  visualization->DrawPoints(pcl2_in_ego_frame_with_noise, icp_cov::kColorCyan);
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
  auto pcl_alignment = icp_cov::PclAlignment::Instance();
  pcl_alignment->set_eigen_pcl1(pcl1_in_world_frame_with_noise);
  pcl_alignment->set_eigen_pcl2(pcl2_in_world_frame_with_noise);
  pcl_alignment->set_icp_transform_init(icp_transform_init);
  pcl_alignment->Align();
  Eigen::Affine3d icp_transform_est = pcl_alignment->icp_transform_est();
  std::cout << "icp_transform_est: "
            << icp_cov::utils::R2ypr(icp_transform_est.rotation()).transpose()
            << "; " << icp_transform_est.translation().transpose() << std::endl;
  std::cout << "icp_fitness_score: " << pcl_alignment->icp_fitness_score()
            << std::endl;
  std::vector<Eigen::Vector3d> pcl1_aligned_in_world_frame =
      pcl_alignment->eigen_pcl1_aligned();
  std::vector<Eigen::Vector3d> pcl1_aligned_in_ref_frame;
  icp_cov::utils::TransformPoints(pcl1_aligned_in_world_frame,
                                  data_generator->ego_pose2().inverse(),
                                  pcl1_aligned_in_ref_frame);
  visualization->DrawPoints(pcl1_aligned_in_ref_frame, icp_cov::kColorWhite);

  // step 3: calc icp covariance
  // true / hessian / proposed

  // step 4: visualization
  visualization->Show();

  return 0;
}
