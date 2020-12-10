#include "icp_covariance.h"

#include <pcl/pcl_config.h>

#include <iostream>

#include "color.h"
#include "data_generator.h"
#include "pcl_alignment.h"
#include "utils.h"
#include "visualization.h"

namespace icp_cov {
IcpCovariance* IcpCovariance::icp_covariance_ = nullptr;

IcpCovariance* IcpCovariance::Instance() {
  if (icp_covariance_ == nullptr) {
    icp_covariance_ = new IcpCovariance();
  }
  return icp_covariance_;
}

void IcpCovariance::CalculateIcpCov() {
  IcpCovFromMonteCarlo();
  IcpCovFromHessian();
  IcpCovFromCostFunction();
}

void IcpCovariance::IcpCovFromMonteCarlo() {
  constexpr int kIterationsForMonteCarlo = 300;
  // each row is a yprxyz
  Eigen::Matrix<double, kIterationsForMonteCarlo, 6> yprxyzs;
  for (int i = 0; i < kIterationsForMonteCarlo; ++i) {
    // step 1: generate data
    // pcl
    auto data_generator = icp_cov::DataGenerator::Instance();
    data_generator->Generate();
    auto pcl1_in_world_frame_with_noise =
        data_generator->pcl1_in_world_frame_with_noise();
    auto pcl2_in_world_frame_with_noise =
        data_generator->pcl2_in_world_frame_with_noise();
    // icp transform
    Eigen::Affine3d icp_transform_gt = data_generator->icp_transform();
    Eigen::Affine3d icp_transform_init = data_generator->icp_transform_init();
    std::cout << "icp_transform_gt: "
              << icp_cov::utils::R2ypr(icp_transform_gt.rotation()).transpose()
              << "; " << icp_transform_gt.translation().transpose()
              << std::endl;
    std::cout
        << "icp_transform_init: "
        << icp_cov::utils::R2ypr(icp_transform_init.rotation()).transpose()
        << "; " << icp_transform_init.translation().transpose() << std::endl;

    // step 2: icp
    auto pcl_alignment = icp_cov::PclAlignment::Instance();
    pcl_alignment->set_eigen_pcl1(pcl1_in_world_frame_with_noise);
    pcl_alignment->set_eigen_pcl2(pcl2_in_world_frame_with_noise);
    pcl_alignment->set_icp_transform_init(icp_transform_init);
    pcl_alignment->Align();
    Eigen::Affine3d icp_transform_est = pcl_alignment->icp_transform_est();
    Eigen::Vector3d ypr = icp_cov::utils::R2ypr(icp_transform_est.rotation());
    Eigen::Vector3d xyz = icp_transform_est.translation();
    if (ypr.norm() > 90) {
      // std::cout << "###############################################" <<
      // std::endl; auto pcl1_in_ref_frame =
      // data_generator->pcl1_in_ref_frame(); auto pcl2_in_ref_frame =
      // data_generator->pcl2_in_ref_frame(); auto pcl1_in_ref_frame_with_noise
      // =
      //     data_generator->pcl1_in_ref_frame_with_noise();
      // auto pcl2_in_ref_frame_with_noise =
      //     data_generator->pcl2_in_ref_frame_with_noise();
      // auto visualization = icp_cov::Visualization::Instance();
      // visualization->ResetCanvas();
      // visualization->DrawPoints(pcl1_in_ref_frame, icp_cov::kColorRed);
      // visualization->DrawPoints(pcl2_in_ref_frame, icp_cov::kColorGreen);
      // visualization->DrawPoints(pcl1_in_ref_frame_with_noise,
      //                           icp_cov::kColorPink);
      // visualization->DrawPoints(pcl2_in_ref_frame_with_noise,
      //                           icp_cov::kColorCyan);
      // std::vector<Eigen::Vector3d> pcl1_aligned_in_world_frame =
      //     pcl_alignment->eigen_pcl1_aligned();
      // std::vector<Eigen::Vector3d> pcl1_aligned_in_ref_frame;
      // icp_cov::utils::TransformPoints(pcl1_aligned_in_world_frame,
      //                                 data_generator->ref_pose().inverse(),
      //                                 pcl1_aligned_in_ref_frame);
      // visualization->DrawPoints(pcl1_aligned_in_ref_frame,
      //                           icp_cov::kColorWhite);
      // const int kScale = 1000;
      // const std::string image_name =
      //     std::to_string(i) + "_" +
      //     std::to_string(static_cast<int>(ypr(0) * kScale)) + "_" +
      //     std::to_string(static_cast<int>(ypr(1) * kScale)) + "_" +
      //     std::to_string(static_cast<int>(ypr(2) * kScale)) + "_" +
      //     std::to_string(static_cast<int>(xyz(0) * kScale)) + "_" +
      //     std::to_string(static_cast<int>(xyz(1) * kScale)) + "_" +
      //     std::to_string(static_cast<int>(xyz(2) * kScale)) + ".png";
      // visualization->Save(image_name);
      // pcl_alignment->Debug();
      // 如果icp出的ypr出现180度歧义性,则放弃这次结果
      std::cout << "bad "
                   "simulation#################################################"
                   "##############################"
                << std::endl;
      std::cout << "icp_transform_est: " << ypr.transpose() << "; "
                << xyz.transpose() << std::endl;
      std::cout << "icp_fitness_score: " << pcl_alignment->icp_fitness_score()
                << std::endl;
      i--;
      continue;
    }
    std::cout << "icp_transform_est: " << ypr.transpose() << "; "
              << xyz.transpose() << std::endl;
    std::cout << "icp_fitness_score: " << pcl_alignment->icp_fitness_score()
              << std::endl;
    // step 3: save icp_transform_est
    // TODO(clssucceed@gmail.com): icp_transform_est to yprxyz using
    // sophus(SE(3) -> se(3))
    Eigen::Matrix<double, 6, 1> yprxyz;
    yprxyz.head(3) = ypr;
    yprxyz.tail(3) = xyz;
    yprxyzs.row(i) = yprxyz.transpose();
  }
  icp_cov_from_monte_carlo_ = icp_cov::utils::Covariance(yprxyzs);
}

void IcpCovariance::IcpCovFromHessian() {}

void IcpCovariance::IcpCovFromCostFunction() {}

}  // namespace icp_cov