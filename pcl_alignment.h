#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
class PclAlignment {
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;
  using EigenPoint = Eigen::Vector3d;
  using EigenPointCloud = std::vector<EigenPoint>;

  static constexpr int kMaxIterations = 15;

 public:
  static PclAlignment* Instance();

 public:
  void set_eigen_pcl1(const EigenPointCloud& pcl1) {
    eigen_pcl1_ = pcl1;
    EigenPclToPcl(pcl1, pcl1_);
  }
  void set_eigen_pcl2(const EigenPointCloud& pcl2) {
    eigen_pcl2_ = pcl2;
    EigenPclToPcl(pcl2, pcl2_);
  }
  void set_icp_transform_init(const Eigen::Affine3d& icp_transform_init) {
    icp_transform_init_ = icp_transform_init;
  }
  void set_pcl1_pose(const Eigen::Affine3d& pcl1_pose) {
    pcl1_pose_ = pcl1_pose;
  }
  void set_pcl2_pose(const Eigen::Affine3d& pcl2_pose) {
    pcl2_pose_ = pcl2_pose;
  }
  Eigen::Affine3d icp_transform_init() const { return icp_transform_init_; }
  Eigen::Affine3d icp_transform_est() const { return icp_transform_est_; }
  double icp_fitness_score() const { return icp_fitness_score_; }
  const EigenPointCloud& eigen_pcl1() const { return eigen_pcl1_; }
  const EigenPointCloud& eigen_pcl1_aligned() const {
    return eigen_pcl1_aligned_;
  }
  const EigenPointCloud& eigen_pcl2() const { return eigen_pcl2_; }
  void Align();
  void Debug();

 private:
  static PclAlignment* pcl_alignment_;
  PclAlignment() { pcl1_aligned_.reset(new PointCloudT); };

 private:
  void Downsample();
  void DetectKeyPoint();
  void DetectKeyPoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output);
  void DetectISS(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output);
  void DetectEdgePoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output);
  void EigenPclToPcl(const EigenPointCloud& eigen_pcl, PointCloudT::Ptr& pcl);
  void PclToEigenPcl(const PointCloudT::Ptr& pcl, EigenPointCloud& eigen_pcl);
  void Visualization();

 private:
  // all points are in world frame
  EigenPointCloud eigen_pcl1_;
  EigenPointCloud eigen_pcl1_aligned_;
  EigenPointCloud eigen_pcl2_;
  PointCloudT::Ptr pcl1_ = nullptr;
  PointCloudT::Ptr pcl1_aligned_ = nullptr;
  PointCloudT::Ptr pcl2_ = nullptr;
  PointCloudT::Ptr pcl1_downsampled_ = nullptr;
  PointCloudT::Ptr pcl2_downsampled_ = nullptr;
  PointCloudT::Ptr pcl1_key_points_ = nullptr;
  PointCloudT::Ptr pcl2_key_points_ = nullptr;
  
  Eigen::Affine3d icp_transform_init_;
  Eigen::Affine3d icp_transform_est_;
  double icp_fitness_score_ = -1.0;

  Eigen::Affine3d pcl1_pose_;
  Eigen::Affine3d pcl2_pose_;
};
}  // namespace icp_cov