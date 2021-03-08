#include "pcl_alignment.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>

#include "utils.h"

#include "config/config.h"

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
  Downsample();
  DetectKeyPoint();
  pcl1_aligned_.reset(new PointCloudT);
  assert(pcl1_aligned_);
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(kMaxIterations);
  icp.setInputSource(pcl1_downsampled_);
  icp.setInputTarget(pcl2_downsampled_);
  icp.align(*pcl1_aligned_, icp_transform_init_.matrix().cast<float>());
  assert(icp.hasConverged());
  PclToEigenPcl(pcl1_aligned_, eigen_pcl1_aligned_);
  icp_transform_est_ = icp.getFinalTransformation().cast<double>();
  icp_fitness_score_ = icp.getFitnessScore();
}

void PclAlignment::Downsample() {
  assert(pcl1_);
  assert(pcl2_);
  auto config = icp_cov::Config::Instance();
  if (config->kDownsample) {
    pcl::VoxelGrid<PointT> downsample1;
    downsample1.setInputCloud(pcl1_);
    downsample1.setLeafSize(config->kLeafSize, config->kLeafSize, config->kLeafSize);
    pcl1_downsampled_.reset(new PointCloudT);
    downsample1.filter(*pcl1_downsampled_);
    
    pcl::VoxelGrid<PointT> downsample2;
    downsample2.setInputCloud(pcl2_);
    downsample2.setLeafSize(config->kLeafSize, config->kLeafSize, config->kLeafSize);
    pcl2_downsampled_.reset(new PointCloudT);
    downsample2.filter(*pcl2_downsampled_);
  } else {
    pcl1_downsampled_ = pcl1_;
    pcl2_downsampled_ = pcl2_;  
  }
  std::cout << "downsampled_pcl_size: " << pcl1_downsampled_->size() << "/" << pcl2_downsampled_->size() << std::endl;
}

void PclAlignment::DetectKeyPoint() {
  DetectKeyPoint(pcl1_downsampled_, pcl1_key_points_);
  DetectKeyPoint(pcl2_downsampled_, pcl2_key_points_);
}

void PclAlignment::DetectKeyPoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  auto config = icp_cov::Config::Instance();
  const double cloud_resolution (config->kLaserHorizontalAngleResolution);

  //
  // Compute the ISS 3D keypoints - By first performing the Boundary Estimation
  //
  pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (6 * cloud_resolution);
  iss_detector.setNonMaxRadius (4 * cloud_resolution);
  iss_detector.setNormalRadius (4 * cloud_resolution);
  iss_detector.setBorderRadius (4 * cloud_resolution);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (5);
  iss_detector.setAngleThreshold (static_cast<float> (M_PI) / 3.0);
  iss_detector.setNumberOfThreads (1);
  iss_detector.setInputCloud (pcl_input);

  if (!pcl_output) {
    pcl_output.reset(new PointCloudT);
  }
  assert(pcl_output);
  iss_detector.compute (*pcl_output);
}

void PclAlignment::Debug() {
  icp_cov::utils::PrintPoints(eigen_pcl1_, "pcl1");
  icp_cov::utils::PrintPoints(eigen_pcl2_, "pcl2");
  icp_cov::utils::PrintPoints(eigen_pcl1_aligned_, "pcl1_aligned");
}
}  // namespace icp_cov