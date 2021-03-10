#include "pcl_alignment.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>

#include "utils.h"
#include "time_analysis.h"
#include "config/config.h"

#include <thread>
#include <chrono>
#include <unordered_set>
#include <unordered_map>

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
  icp_cov::TimeAnalysis all_cost;
  icp_cov::TimeAnalysis downsample_cost;
  Downsample();
  downsample_cost.Stop("downsample");
  icp_cov::TimeAnalysis detect_key_point_cost;
  DetectKeyPoint();
  detect_key_point_cost.Stop("detect key point");
  icp_cov::TimeAnalysis icp_cost;
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
  icp_cost.Stop("icp");
  all_cost.Stop("all");
  Visualization();
}

void PclAlignment::Downsample() {
  auto config = icp_cov::Config::Instance();
  if (config->kDownsample) {
    pcl1_downsampled_.reset(new PointCloudT);
    pcl2_downsampled_.reset(new PointCloudT);
    DownsampleWithPclApi(pcl1_, pcl1_downsampled_);
    DownsampleWithPclApi(pcl2_, pcl2_downsampled_);
    // DownsampleOutputOrganizedPcl(pcl1_, pcl1_pose_, config->kTargetSize, pcl1_downsampled_);
    // DownsampleOutputOrganizedPcl(pcl2_, pcl2_pose_, config->kTargetSize, pcl2_downsampled_);
  } else {
    pcl1_downsampled_ = pcl1_;
    pcl2_downsampled_ = pcl2_;
  }
  std::cout << "downsampled_pcl_size: " << pcl1_downsampled_->size() << "/" << pcl2_downsampled_->size() << std::endl;
}

void PclAlignment::DownsampleWithPclApi(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  assert(pcl_input);
  assert(pcl_output);
  auto config = icp_cov::Config::Instance();
  pcl::VoxelGrid<PointT> downsample;
  downsample.setInputCloud(pcl_input);
  downsample.setLeafSize(config->kLeafSize, config->kLeafSize, config->kLeafSize);
  downsample.filter(*pcl_output);
}

void PclAlignment::DetectKeyPoint() {
  auto config = icp_cov::Config::Instance();
  if (config->kEdgeDetection) {
    pcl1_key_points_.reset(new PointCloudT);
    pcl2_key_points_.reset(new PointCloudT);
    DetectKeyPoint(pcl1_downsampled_, pcl1_key_points_);
    DetectKeyPoint(pcl2_downsampled_, pcl2_key_points_);
    // DetectContour(pcl1_downsampled_, pcl1_pose_, config->kTargetSize, pcl1_key_points_);
    // DetectContour(pcl2_downsampled_, pcl2_pose_, config->kTargetSize, pcl2_key_points_);
  } else {
    pcl1_key_points_ = pcl1_downsampled_;
    pcl2_key_points_ = pcl2_downsampled_;
  }
  std::cout << "key_points_size: " << pcl1_key_points_->size() << "/" << pcl2_key_points_->size() << std::endl;
}

void PclAlignment::DetectKeyPoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  auto config = icp_cov::Config::Instance();
  if (config->kApproxMevr) {
    DetectEdgePointApproxMevr(pcl_input, pcl_output);
  } else {
    DetectEdgePoint(pcl_input, pcl_output);
  }
}

void PclAlignment::DetectEdgePoint(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  // step 1: preparation: calc nnn(nearest neighbor number) + mevr(minimum_eigen_value_ratio)
  // step 1.1: some initialization
  auto config = icp_cov::Config::Instance();
  TimeAnalysis construct_kdtree_cost;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (pcl_input);
  construct_kdtree_cost.Stop("construct_kdtree_cost");
  const int pcl_size = pcl_input->size();
  std::multimap<int, int> map_nnn_to_index;
  std::multimap<float, int, std::greater<float>> map_mevr_to_index;
  std::vector<int> nnn_vec;
  std::vector<float> mevr_vec;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  int max_nnn = -1;
  float max_mevr = -1.0;
  const float radius = config->kLeafSize * config->kRadiusRatio;
  TimeAnalysis calc_mevr_and_nnn_cost; 
  // step 1.2: reformat pcl_input to speed up pca
  TimeAnalysis reformat_pcl_input_cost;
  std::vector<Eigen::Vector3d> input_points_vector(pcl_size);
  // ppt: p * p.transpose(); p is input point
  std::vector<Eigen::Matrix3d> input_points_ppt_vector(pcl_size); 
  for (int i = 0; i < pcl_size; ++i) {
    const auto& point = pcl_input->at(i);
    Eigen::Vector3d& point_eigen = input_points_vector.at(i);
    point_eigen(0) = point.x;
    point_eigen(1) = point.y;
    point_eigen(2) = point.z;
    input_points_ppt_vector.at(i) = point_eigen * point_eigen.transpose(); 
  }
  reformat_pcl_input_cost.Stop("reformat_pcl_input_cost");
  // step 1.3: calc mevr and nnn 
  Eigen::Vector3d sum;
  Eigen::Matrix3d cov;
  for (int i = 0; i < pcl_size; ++i) {
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    TimeAnalysis kdtree_search_cost;
    const int nnn = kdtree.radiusSearch(pcl_input->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    kdtree_search_cost.Stop("kdtree_search_cost");
    assert(nnn > 0); 
    assert(nnn == pointIdxRadiusSearch.size());
    assert(nnn == pointRadiusSquaredDistance.size());
    cov.setZero();
    sum.setZero();
    TimeAnalysis pca_cost;
    TimeAnalysis calc_cov_cost;
    for (const auto idx : pointIdxRadiusSearch) {
      sum += input_points_vector.at(idx);
      cov += input_points_ppt_vector.at(idx);
    }
    cov += nnn * input_points_ppt_vector.at(i); 
    cov -= input_points_vector.at(i) * sum.transpose();
    cov -= sum * input_points_vector.at(i).transpose();
    calc_cov_cost.Stop("calc_cov_cost");
    TimeAnalysis calc_eigen_values_cost;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov);
    const Eigen::Vector3d eigen_values = solver.eigenvalues();
    calc_eigen_values_cost.Stop("calc_eigen_values_cost");
    pca_cost.Stop("pca_cost"); 
    assert(eigen_values(1) + eigen_values(2) > 1.0e-6);
    const float mevr = eigen_values(0) / (eigen_values(1) + eigen_values(2));
    map_nnn_to_index.insert({nnn, i});
    map_mevr_to_index.insert({mevr, i});
    max_nnn = std::max(max_nnn, nnn);
    max_mevr = std::max(max_mevr, mevr);
    nnn_vec.push_back(nnn);
    mevr_vec.push_back(mevr);
  }
  calc_mevr_and_nnn_cost.Stop("calc_mevr_and_nnn_cost");
  assert(map_nnn_to_index.size() == pcl_size);
  assert(map_mevr_to_index.size() == pcl_size);
  assert(nnn_vec.size() == pcl_size);
  assert(mevr_vec.size() == pcl_size);
  // step 2: detect point: mevr is large enough || nnn is small enough
  assert(pcl_output);
  std::unordered_set<int> selected_indexes;
  TimeAnalysis mevr_select_cost;
  if (config->kMevrSelect) {
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
  }
  const int mevr_select_num = selected_indexes.size();
  std::cout << "mevr select " << mevr_select_num << std::endl;
  mevr_select_cost.Stop("mevr_select_cost");
  TimeAnalysis nnn_select_cost;
  if (config->kNnnSelect) {
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
        break;
      }
      selected_indexes.insert(item.second);
    }
  }
  const int nnn_select_num = selected_indexes.size() - mevr_select_num;
  std::cout << "nnn select " << nnn_select_num << std::endl;
  nnn_select_cost.Stop("nnn_select_cost");
  TimeAnalysis make_output_cost;
  assert(pcl_output);
  for (const auto& index : selected_indexes) {
    pcl_output->push_back(pcl_input->at(index));
  }
  make_output_cost.Stop("make_output_cost");
}

void PclAlignment::DetectEdgePointApproxMevr(PointCloudT::ConstPtr pcl_input, PointCloudT::Ptr pcl_output) {
  // step 1: preparation: calc nnn(nearest neighbor number) + mevr(minimum_eigen_value_ratio)
  // step 1.1: some initialization
  auto config = icp_cov::Config::Instance();
  TimeAnalysis construct_kdtree_cost;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (pcl_input);
  construct_kdtree_cost.Stop("construct_kdtree_cost");
  const int pcl_size = pcl_input->size();
  std::multimap<int, int> map_nnn_to_index;
  std::multimap<float, int, std::greater<float>> map_mevr_to_index;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  int max_nnn = -1;
  float max_mevr = -1.0;
  const float radius = config->kLeafSize * config->kRadiusRatio;
  TimeAnalysis calc_mevr_and_nnn_cost; 
  // step 1.2: reformat pcl_input to speed up pca
  TimeAnalysis reformat_pcl_input_cost;
  std::vector<Eigen::Vector3d> input_points_vector(pcl_size);
  // psq: (px*px, py*py, pz*pz); p is input point
  std::vector<Eigen::Vector3d> input_points_psq_vector(pcl_size); 
  for (int i = 0; i < pcl_size; ++i) {
    const auto& point = pcl_input->at(i);
    input_points_vector.at(i)(0) = point.x;
    input_points_vector.at(i)(1) = point.y;
    input_points_vector.at(i)(2) = point.z;
    input_points_psq_vector.at(i)(0) = point.x * point.x;
    input_points_psq_vector.at(i)(1) = point.y * point.y;
    input_points_psq_vector.at(i)(2) = point.z * point.z;
  }
  reformat_pcl_input_cost.Stop("reformat_pcl_input_cost");
  // step 1.3: calc mevr and nnn 
  Eigen::Vector3d sum;
  Eigen::Vector3d cov_diag;
  for (int i = 0; i < pcl_size; ++i) {
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    TimeAnalysis kdtree_search_cost;
    const int nnn = kdtree.radiusSearch(pcl_input->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    kdtree_search_cost.Stop("kdtree_search_cost");
    assert(nnn > 0); 
    assert(nnn == pointIdxRadiusSearch.size());
    assert(nnn == pointRadiusSquaredDistance.size());
    cov_diag.setZero();
    sum.setZero();
    TimeAnalysis calc_mevr_cost;
    TimeAnalysis calc_cov_cost;
    for (const auto idx : pointIdxRadiusSearch) {
      sum += input_points_vector.at(idx);
      cov_diag += input_points_psq_vector.at(idx);
    }
    cov_diag += nnn * input_points_psq_vector.at(i); 
    cov_diag(0) -= sum(0) * input_points_vector.at(i)(0) * 2;
    cov_diag(1) -= sum(1) * input_points_vector.at(i)(1) * 2;
    cov_diag(2) -= sum(2) * input_points_vector.at(i)(2) * 2;
    const double min_cov_diag = std::min(cov_diag(0), std::min(cov_diag(1), cov_diag(2)));
    calc_cov_cost.Stop("calc_cov_cost");
    const double cov_diag_sum = cov_diag(0) + cov_diag(1) + cov_diag(2);
    const float mevr = min_cov_diag / cov_diag_sum;
    calc_mevr_cost.Stop("calc_mevr_cost");
    map_nnn_to_index.insert({nnn, i});
    map_mevr_to_index.insert({mevr, i});
    max_nnn = std::max(max_nnn, nnn);
    max_mevr = std::max(max_mevr, mevr);
  }
  calc_mevr_and_nnn_cost.Stop("calc_mevr_and_nnn_cost");
  assert(map_nnn_to_index.size() == pcl_size);
  assert(map_mevr_to_index.size() == pcl_size);
  // step 2: detect point: mevr is large enough || nnn is small enough
  assert(pcl_output);
  std::unordered_set<int> selected_indexes;
  TimeAnalysis mevr_select_cost;
  if (config->kMevrSelect) {
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
  }
  const int mevr_select_num = selected_indexes.size();
  std::cout << "mevr select " << mevr_select_num << std::endl;
  mevr_select_cost.Stop("mevr_select_cost");
  TimeAnalysis nnn_select_cost;
  if (config->kNnnSelect) {
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
        break;
      }
      selected_indexes.insert(item.second);
    }
  }
  const int nnn_select_num = selected_indexes.size() - mevr_select_num;
  std::cout << "nnn select " << nnn_select_num << std::endl;
  nnn_select_cost.Stop("nnn_select_cost");
  TimeAnalysis make_output_cost;
  assert(pcl_output);
  for (const auto& index : selected_indexes) {
    pcl_output->push_back(pcl_input->at(index));
  }
  make_output_cost.Stop("make_output_cost");
}

void PclAlignment::DetectContour(PointCloudT::ConstPtr pcl_input, const Eigen::Affine3d& target_pose, const Eigen::Vector3d& target_size, PointCloudT::Ptr pcl_output) {
  auto config = icp_cov::Config::Instance();
  // step 1: init grid info
  const double small_grid_size = config->kLeafSize;
  const double large_grid_size = config->kLeafSize * config->kRadiusRatio;
  const int paddle_size = 2;
  assert(small_grid_size > 1.0e-3);
  const int small_grid_x_len = static_cast<int>(target_size(0) / small_grid_size) + 1 + 2 * paddle_size; 
  const int small_grid_y_len = static_cast<int>(target_size(1) / small_grid_size) + 1 + 2 * paddle_size; 
  const int small_grid_z_len = static_cast<int>(target_size(2) / small_grid_size) + 1 + 2 * paddle_size; 
  const int large_grid_x_len = static_cast<int>(target_size(0) / large_grid_size) + 1; 
  const int large_grid_y_len = static_cast<int>(target_size(1) / large_grid_size) + 1; 
  const int large_grid_z_len = static_cast<int>(target_size(2) / large_grid_size) + 1; 
  std::unordered_map<int, std::list<int>> map_small_grid_index_to_point_indexes;
  std::unordered_map<int, std::list<int>> map_large_grid_index_to_point_indexes;
  // step 2: assign each point to grid
  // step 2.1: transform point cloud to target frame (O(n))
  // step 2.1.1: move target_pose from rear wheel center to rear top left corner in order to ensure every point in the first quadrant
  const Eigen::Vector3d rear_top_left_corner_in_target_frame(0, -0.5 * target_size(1), -target_size(2));
  const Eigen::Vector3d rear_top_left_corner_in_ref_frame = target_pose * rear_top_left_corner_in_target_frame;
  const Eigen::Affine3d target_pose_new = icp_cov::utils::RtToAffine3d(target_pose.rotation(), rear_top_left_corner_in_ref_frame); 
  // step 2.1.2: transform points
  TimeAnalysis transform_points_cost;
  PointCloudT::Ptr pcl_in_target_frame(new PointCloudT);
  pcl::transformPointCloud(*pcl_input, *pcl_in_target_frame, target_pose_new.inverse());
  assert(pcl_in_target_frame->size() == pcl_input->size());
  transform_points_cost.Stop("transform_points_cost");
  // step 2.2: assign points: O(n) 
  const int pcl_size = pcl_input->size();
  TimeAnalysis assign_points_cost;
  for (int i = 0; i < pcl_size; ++i) {
    const auto& point = pcl_in_target_frame->at(i);
    const int small_grid_x = std::max(0, std::min(small_grid_x_len - 1, static_cast<int>(point.x / small_grid_size) + paddle_size)); 
    const int small_grid_y = std::max(0, std::min(small_grid_y_len - 1, static_cast<int>(point.y / small_grid_size) + paddle_size)); 
    const int small_grid_z = std::max(0, std::min(small_grid_z_len - 1, static_cast<int>(point.z / small_grid_size) + paddle_size)); 
    const int small_grid_index = small_grid_x * small_grid_y_len * small_grid_z_len + small_grid_y * small_grid_z_len + small_grid_z;
    map_small_grid_index_to_point_indexes[small_grid_index].push_back(i);

    const int large_grid_x = std::max(0, std::min(large_grid_x_len - 1, static_cast<int>(point.x / large_grid_size))); 
    const int large_grid_y = std::max(0, std::min(large_grid_y_len - 1, static_cast<int>(point.y / large_grid_size))); 
    const int large_grid_z = std::max(0, std::min(large_grid_z_len - 1, static_cast<int>(point.z / large_grid_size))); 
    const int large_grid_index = large_grid_x * large_grid_y_len * large_grid_z_len + large_grid_y * large_grid_z_len + large_grid_z;
    map_large_grid_index_to_point_indexes[large_grid_index].push_back(i);
  }
  assign_points_cost.Stop("assign_points_cost");
  // step 3: analyze each grid info
  // step 3.1: analyze mevr for each large grid
  TimeAnalysis analyze_mevr_cost;
  std::multimap<float, int, std::greater<float>> map_mevr_to_large_grid_index;
  float max_mevr = -1;
  for (const auto& large_grid_info : map_large_grid_index_to_point_indexes) {
    const int large_grid_index = large_grid_info.first;
    const auto large_grid_point_indexes = large_grid_info.second;
    const int large_grid_num = large_grid_point_indexes.size();
    if (large_grid_num > 4) {
      const Eigen::Vector3d eigen_values = EigenValuesOfPclDistribution(pcl_input, large_grid_point_indexes);
      assert(eigen_values(1) + eigen_values(2) > 1.0e-6);
      const float mevr = eigen_values(0) / (eigen_values(1) + eigen_values(2));
      map_mevr_to_large_grid_index.insert({mevr, large_grid_index});
      max_mevr = std::max(max_mevr, mevr);
    }
  } 
  std::cout << "map_large_grid_index_to_point_indexes.size = " 
            << map_large_grid_index_to_point_indexes.size() << std::endl;
  analyze_mevr_cost.Stop("analyze_mevr_cost");
  // step 3.2: analyze nnn for each small grid (O(n))
  TimeAnalysis analyze_nnn_cost;
  std::multimap<double, int> map_nnn_to_small_grid_index;
  double max_nnn = -1;
  // const double dist_th_for_hd = config->kLeafSize / (config->kLaserHorizontalAngleResolution * config->kDegToRad);
  // const double dist_th_for_vd = config->kLeafSize / (config->kLaserVerticalAngleResolution * config->kDegToRad);
  // const double dist_sq_th = dist_th_for_hd * dist_th_for_vd;
  // std::cout << "$$$$$$$$$$$$" << dist_th_for_hd << "/" << dist_th_for_vd << std::endl;
  for (const auto& small_grid_info : map_small_grid_index_to_point_indexes) {
    const int small_grid_index = small_grid_info.first;
    const int point_index = small_grid_info.second.front();
    const auto& current_point = pcl_input->at(point_index);
    double nnn = 0;
    double min_dist_sq = 1.0;
    for (int x = -1; x < 2; ++x) {
      for (int y = -1; y < 2; ++y) {
        for (int z = -1; z < 2; ++z) {
          const int neighbor_small_grid_index = small_grid_index + x * small_grid_y_len * small_grid_z_len + y * small_grid_z_len + z;
          if (map_small_grid_index_to_point_indexes.count(neighbor_small_grid_index)) {
            nnn += map_small_grid_index_to_point_indexes[neighbor_small_grid_index].size();
            const auto& neighbor_point = pcl_input->at(
              map_small_grid_index_to_point_indexes[neighbor_small_grid_index].front()); 
            if (!(0 == x && 0 == y && 0 == z)) {
              min_dist_sq = std::min(min_dist_sq, 
                std::pow(current_point.x - neighbor_point.x, 2) + 
                std::pow(current_point.y - neighbor_point.y, 2) + 
                std::pow(current_point.z - neighbor_point.z, 2));
            }
          }
        }
      }
    }    
    // const double dist_sq = current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z; 
    // nnn *= std::max(dist_sq, dist_sq_th);
    // nnn *= dist_sq;
    nnn *= min_dist_sq; 
    map_nnn_to_small_grid_index.insert({nnn, small_grid_index});
    max_nnn = std::max(max_nnn, nnn);
  } 
  analyze_nnn_cost.Stop("analyze_nnn_cost");
  // step 4: select valid small_grid points as contour points (mevr is large enough || nnn is small enough) 
  std::unordered_set<int> selected_indexes;
  TimeAnalysis mevr_select_cost;
  if (config->kMevrSelect) {
    const float mevr_th = std::max(config->kMevrThRatio * max_mevr, config->kMevrThLowBound);
    float last_mevr = map_mevr_to_large_grid_index.begin()->first;
    std::cout << "max_mevr = " << max_mevr << ", mevr_th = " << mevr_th << std::endl;
    for (const auto& item : map_mevr_to_large_grid_index) {
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
      const int large_grid_index = item.second;
      const auto point_indexes = map_large_grid_index_to_point_indexes[large_grid_index];
      for (const auto& point_index : point_indexes) {
        selected_indexes.insert(point_index);
      }
      last_mevr = mevr;
    }
  }
  const int mevr_select_num = selected_indexes.size();
  std::cout << "mevr select " << mevr_select_num << std::endl;
  mevr_select_cost.Stop("mevr_select_cost");
  TimeAnalysis nnn_select_cost;
  if (config->kNnnSelect) {
    const int nnn_th = config->kNnnThRatio * max_nnn;
    std::cout << "max_nnn = " << max_nnn << ", nnn_th = " << nnn_th << std::endl;
    for (const auto& item : map_nnn_to_small_grid_index) {
      const int nnn = item.first;
      if (nnn > nnn_th) {
        std::cout << "nnn > nnn_th: " << nnn << " > " << nnn_th << std::endl;
        break;
      }
      if (selected_indexes.size() >= config->kAllSelectNumUpBound) {
        std::cout << "selected_indexes.size() >= 200: " << selected_indexes.size() << "/" << nnn  << std::endl;
        break;
      }
      const int small_grid_index = item.second;
      const auto point_indexes = map_small_grid_index_to_point_indexes[small_grid_index];
      for (const auto& point_index : point_indexes) {
        selected_indexes.insert(point_index);
      }
    }
  }
  const int nnn_select_num = selected_indexes.size() - mevr_select_num;
  std::cout << "nnn select " << nnn_select_num << std::endl;
  nnn_select_cost.Stop("nnn_select_cost");
  // step 5: make output
  TimeAnalysis make_output_cost;
  assert(pcl_output);
  for (const auto& index : selected_indexes) {
    pcl_output->push_back(pcl_input->at(index));
  }
  make_output_cost.Stop("make_output_cost");
}


Eigen::Vector3d PclAlignment::EigenValuesOfPclDistribution(PointCloudT::ConstPtr pcl, const std::list<int> indexes) {
  assert(indexes.size() > 4);
  Eigen::Vector3d mean(0.0, 0.0, 0.0);
  for (const auto idx : indexes) {
    const auto& point = pcl->at(idx);
    const Eigen::Vector3d p(point.x, point.y, point.z);
    mean += p; 
  }
  mean /= indexes.size();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto idx : indexes) {
    const auto& point = pcl->at(idx);
    const Eigen::Vector3d p(point.x, point.y, point.z);
    const Eigen::Vector3d diff = mean - p;
    cov += diff * diff.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov);
  return solver.eigenvalues(); 
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
    // using namespace std::chrono_literals;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
  }
}

void PclAlignment::Debug() {
  icp_cov::utils::PrintPoints(eigen_pcl1_, "pcl1");
  icp_cov::utils::PrintPoints(eigen_pcl2_, "pcl2");
  icp_cov::utils::PrintPoints(eigen_pcl1_aligned_, "pcl1_aligned");
}
}  // namespace icp_cov