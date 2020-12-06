#include "pcl_alignment.h"

#include <pcl/registration/icp.h>

namespace icp_cov {
PclAlignment* PclAlignment::pcl_alignment_ = nullptr;

PclAlignment* PclAlignment::Instance() {
  if (pcl_alignment_ == nullptr) {
    pcl_alignment_ = new PclAlignment();
  }
  return pcl_alignment_;
}

void PclAlignment::EigenPclToPcl(const EigenPointCloud& eigen_pcl,
                                 PointCloudT::Ptr& pcl) {
  pcl.reset(new PointCloudT);
  assert(pcl);
  for (const auto& eigen_point : eigen_pcl) {
    pcl->points.push_back(PointT{static_cast<float>(eigen_point(0)),
                                 static_cast<float>(eigen_point(1)),
                                 static_cast<float>(eigen_point(2))});
  }
}

void PclAlignment::PclToEigenPcl(const PointCloudT::Ptr& pcl,
                                 EigenPointCloud& eigen_pcl) {
  assert(pcl);
  eigen_pcl.clear();
  eigen_pcl.reserve(pcl->size());
  for (const auto& point : pcl->points) {
    eigen_pcl.emplace_back(EigenPoint{static_cast<double>(point.x),
                                      static_cast<double>(point.y),
                                      static_cast<double>(point.z)});
  }
}

// TODO(clssucceed@gmail.com): 尝试2d
// icp(pcl::registration::TransformationEstimation2D)
void PclAlignment::Align() {
  assert(pcl1_);
  assert(pcl2_);
  assert(pcl1_aligned_);
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(kMaxIterations);
  icp.setInputSource(pcl1_);
  icp.setInputTarget(pcl2_);
  icp.align(*pcl1_aligned_, icp_transform_init_.matrix().cast<float>());
  assert(icp.hasConverged());
  PclToEigenPcl(pcl1_aligned_, eigen_pcl1_aligned_);
  icp_transform_est_ = icp.getFinalTransformation().cast<double>();
  icp_fitness_score_ = icp.getFitnessScore();
}
}  // namespace icp_cov