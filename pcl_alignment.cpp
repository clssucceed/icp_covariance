#include "pcl_alignment.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utils.h"

#include "config/config.h"
#include <thread>
#include <chrono>
#include <unordered_set>

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
  icp.setInputSource(pcl1_key_points_);
  icp.setInputTarget(pcl2_key_points_);
  icp.align(*pcl1_aligned_, icp_transform_init_.matrix().cast<float>());
  assert(icp.hasConverged());
  PclToEigenPcl(pcl1_aligned_, eigen_pcl1_aligned_);
  icp_transform_est_ = icp.getFinalTransformation().cast<double>();
  icp_fitness_score_ = icp.getFitnessScore();
  Visualization();
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
  auto config = icp_cov::Config::Instance();
  if (config->kEdgeDetection) {
    pcl1_key_points_.reset(new PointCloudT);
    pcl2_key_points_.reset(new PointCloudT);
    DetectKeyPoint(pcl1_downsampled_, pcl1_key_points_);
    DetectKeyPoint(pcl2_downsampled_, pcl2_key_points_);
  } else {
    pcl1_key_points_ = pcl1_downsampled_;
    pcl2_key_points_ = pcl2_downsampled_;
  }
  std::cout << "key_points_size: " << pcl1_key_points_->size() << "/" << pcl2_key_points_->size() << std::endl;
}

void PclAlignment::DetectKeyPoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  DetectEdgePoint(pcl_input, pcl_output);
}

void PclAlignment::DetectEdgePoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  // step 1: prepare: calc nnn(nearest neighbor number) + mevr(minimum_eigen_value_ratio)
  auto config = icp_cov::Config::Instance();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (pcl_input);
  const int pcl_size = pcl_input->size();
  std::multimap<int, int> map_nnn_to_index;
  std::multimap<float, int, std::greater<float>> map_mevr_to_index;
  std::vector<int> nnn_vec;
  std::vector<float> mevr_vec;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  Eigen::Matrix3d cov;
  int max_nnn = -1;
  float max_mevr = -1.0;
  const float radius = config->kLeafSize * config->kRadiusRatio;
  for (int i = 0; i < pcl_size; ++i) {
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    const auto& current_point = pcl_input->at(i);
    const int nnn = kdtree.radiusSearch(current_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    assert(nnn > 0); 
    assert(nnn == pointIdxRadiusSearch.size());
    assert(nnn == pointRadiusSquaredDistance.size());
    cov.setZero();
    const Eigen::Vector3d cp(current_point.x, current_point.y, current_point.z);
    for (const auto idx : pointIdxRadiusSearch) {
      const auto& neighbor_point = pcl_input->at(idx);
      const Eigen::Vector3d np(neighbor_point.x, neighbor_point.y, neighbor_point.z);
      const Eigen::Vector3d diff = cp - np;
      cov += diff * diff.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov);
    const Eigen::Vector3d eigen_values = solver.eigenvalues(); 
    assert(eigen_values(1) + eigen_values(2) > 1.0e-6);
    const float mevr = eigen_values(0) / (eigen_values(1) + eigen_values(2));
    map_nnn_to_index.insert({nnn, i});
    map_mevr_to_index.insert({mevr, i});
    max_nnn = std::max(max_nnn, nnn);
    max_mevr = std::max(max_mevr, mevr);
    nnn_vec.push_back(nnn);
    mevr_vec.push_back(mevr);
  }
  assert(map_nnn_to_index.size() == pcl_size);
  assert(map_mevr_to_index.size() == pcl_size);
  assert(nnn_vec.size() == pcl_size);
  assert(mevr_vec.size() == pcl_size);
  // step 2: detect point: mevr is large enough || nnn is small enough
  assert(pcl_output);
  std::unordered_set<int> selected_indexes;
  const float mevr_th = std::max(config->kMevrThRatio * max_mevr, config->kMevrThLowBound);
  float last_mevr = map_mevr_to_index.begin()->first;
  std::cout << "max_mevr = " << max_mevr << ", mevr_th = " << mevr_th << std::endl;
  for (const auto& item : map_mevr_to_index) {
    const float mevr = item.first;
    if (mevr < mevr_th) {
      std::cout << "mevr < mevr_th: " << mevr << " < " << mevr_th << std::endl;  
      break;
    }
    if (last_mevr > 5 * mevr) {
      std::cout << "last_mevr > 5 * mevr: " << last_mevr << " > 5 * " << mevr << std::endl;
      break;  
    } 
    if (selected_indexes.size() >= config->kMevrSelectNumUpBound) {
      std::cout << "selected_indexes.size() >= 100: " << selected_indexes.size() << "/" << mevr << std::endl;
      break;
    }
    selected_indexes.insert(item.second);
    last_mevr = mevr;
  }
  const int mevr_select_num = selected_indexes.size();
  std::cout << "mevr select " << mevr_select_num << std::endl;
  const int nnn_th = config->kNnnThRatio * max_nnn;
  std::cout << "max_nnn = " << max_nnn << ", nnn_th = " << nnn_th << std::endl;
  for (const auto& item : map_nnn_to_index) {
    const int nnn = item.first;
    if (nnn > nnn_th) {
      std::cout << "nnn > nnn_th: " << nnn << " > " << nnn_th << std::endl;
      break;
    }
    if (selected_indexes.size() >= config->kAllSelectNumUpBound) {
      std::cout << "selected_indexes.size() >= 200: " << selected_indexes.size() << "/" << nnn  << std::endl;
    }
    selected_indexes.insert(item.second);
  }
  const int nnn_select_num = selected_indexes.size() - mevr_select_num;
  std::cout << "nnn select " << nnn_select_num << std::endl;
  assert(pcl_output);
  for (const auto& index : selected_indexes) {
    pcl_output->push_back(pcl_input->at(index));
  }
}

void PclAlignment::DetectISS(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  auto config = icp_cov::Config::Instance();
  const double cloud_resolution (config->kLaserHorizontalAngleResolution);

  //
  // Compute the ISS 3D keypoints - By first performing the Boundary Estimation
  //
  pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (0.2);
  iss_detector.setNonMaxRadius (0.01);
  // iss_detector.setNormalRadius (4 * cloud_resolution);
  // iss_detector.setBorderRadius (4 * cloud_resolution);
  // iss_detector.setAngleThreshold (static_cast<float> (M_PI) / 3.0);
  iss_detector.setThreshold21 (0.975);
  iss_detector.setThreshold32 (0.975);
  iss_detector.setMinNeighbors (1);
  iss_detector.setNumberOfThreads (1);
  iss_detector.setInputCloud (pcl_input);

  assert(pcl_output);
  iss_detector.compute (*pcl_output);
}

void PclAlignment::Visualization() {
  auto config = icp_cov::Config::Instance();
  if (!config->kPclAlignmentVisualization) return;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("pcl_alignment"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcl1_downsampled_color(pcl1_downsampled_, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (pcl1_downsampled_, pcl1_downsampled_color, "pcl1_downsampled");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pcl1_downsampled");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcl2_downsampled_color(pcl2_downsampled_, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (pcl2_downsampled_, pcl2_downsampled_color, "pcl2_downsampled");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pcl2_downsampled");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcl1_key_points_color(pcl1_key_points_, 255, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (pcl1_key_points_, pcl1_key_points_color, "pcl1_key_points");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl1_key_points");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcl2_key_points_color(pcl2_key_points_, 0, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (pcl2_key_points_, pcl2_key_points_color, "pcl2_key_points");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl2_key_points");

  viewer->addCoordinateSystem (1.0, pcl1_pose_.cast<float>());
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
  }
}

void PclAlignment::Debug() {
  icp_cov::utils::PrintPoints(eigen_pcl1_, "pcl1");
  icp_cov::utils::PrintPoints(eigen_pcl2_, "pcl2");
  icp_cov::utils::PrintPoints(eigen_pcl1_aligned_, "pcl1_aligned");
}
}  // namespace icp_cov