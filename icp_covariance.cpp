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
  Eigen::Matrix<double, kIterationsForMonteCarlo, 1> yaw_rates;
  Eigen::Matrix<double, kIterationsForMonteCarlo, 3> velocitys;
  Eigen::Matrix<double, kIterationsForMonteCarlo, 1> vel_norms;
  Eigen::Matrix<double, kIterationsForMonteCarlo, 1> vel_directions;
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
      // icp_cov::utils::DebugThisSimulation();
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
    // step 4: calc vel and save
    Eigen::Vector3d anchor_point1 = data_generator->ego_pose1().translation();
    Eigen::Vector3d anchor_point1_transformed =
        icp_transform_est * anchor_point1;
    Eigen::Vector3d velocity =
        (anchor_point1_transformed - anchor_point1) / 0.1;
    velocitys.row(i) = velocity.transpose();
    vel_norms(i) = velocity.norm();
    vel_directions(i) = std::atan2(velocity(1), velocity(0)) * kRadToDeg;
    yaw_rates(i) = ypr(0) / 0.1;
    std::cout << "vel: " << velocity(0) << ", " << velocity(1) << ", "
              << velocity.norm() << std::endl;
  }
  std::cout << "icp_cov_from_monte_carlo: " << std::endl;
  icp_cov_from_monte_carlo_ = icp_cov::utils::Covariance(yprxyzs);
  std::cout << "vel_cov_from_monte_carlo: " << std::endl;
  vel_cov_from_monte_carlo_ = icp_cov::utils::Covariance(velocitys);
  std::cout << "vel_norm_cov_from_monte_carlo: " << std::endl;
  vel_norm_cov_from_monte_carlo_ = icp_cov::utils::Covariance(vel_norms);
  std::cout << "vel_direction_cov_from_monte_carlo: " << std::endl;
  vel_direction_cov_from_monte_carlo_ =
      icp_cov::utils::Covariance(vel_directions);
  std::cout << "yaw_rate_cov_from_monte_carlo: " << std::endl;
  yaw_rate_cov_from_monte_carlo_ = icp_cov::utils::Covariance(yaw_rates);
}

void IcpCovariance::IcpCovFromHessian() {}

void IcpCovariance::IcpCovFromCostFunction() {}

}  // namespace icp_cov