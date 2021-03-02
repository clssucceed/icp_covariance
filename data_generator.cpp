#include "data_generator.h"

#include "color.h"
#include "config/config.h"
#include "utils.h"
#include "visualization.h"

namespace icp_cov {

DataGenerator* DataGenerator::data_generator_ = nullptr;

DataGenerator* DataGenerator::Instance() {
  if (data_generator_ == nullptr) {
    data_generator_ = new DataGenerator();
  }
  return data_generator_;
}

void DataGenerator::Generate() {
  Reset();
  // step 1: set ego pose
  auto config = icp_cov::Config::Instance();
  ego_pose1_ = config->kEgoPose1InWorldFrame;
  ego_pose2_ = config->kEgoPose2InWorldFrame;

  // step 2: set target pose
  target_pose1_ = ego_pose1_ * config->kTargetPose1InEgo1Frame;
  target_pose2_ = ego_pose2_ * config->kTargetPose2InEgo2Frame;

  // step 3: generate points in ego frame
  if (icp_cov::Config::Instance()->kGenerate3d) {
    Generate3d();
  } else {
    Generate2d();
  }

  // step 4: Add noise for points in ego frame
  AddNoiseToPoints(pcl1_in_ego_frame_, pcl1_in_ego_frame_with_noise_);
  AddNoiseToPoints(pcl2_in_ego_frame_, pcl2_in_ego_frame_with_noise_);

  // step 5: transform points to world frame
  icp_cov::utils::TransformPoints(pcl1_in_ego_frame_, ego_pose1_,
                                  pcl1_in_world_frame_);
  icp_cov::utils::TransformPoints(pcl2_in_ego_frame_, ego_pose2_,
                                  pcl2_in_world_frame_);
  icp_cov::utils::TransformPoints(pcl1_in_ego_frame_with_noise_, ego_pose1_,
                                  pcl1_in_world_frame_with_noise_);
  icp_cov::utils::TransformPoints(pcl2_in_ego_frame_with_noise_, ego_pose2_,
                                  pcl2_in_world_frame_with_noise_);

  // step 6: transform points to ref frame for visualization
  ref_pose_ = ego_pose2_;
  icp_cov::utils::TransformPoints(pcl1_in_world_frame_, ref_pose_.inverse(),
                                  pcl1_in_ref_frame_);
  icp_cov::utils::TransformPoints(pcl2_in_world_frame_, ref_pose_.inverse(),
                                  pcl2_in_ref_frame_);
  icp_cov::utils::TransformPoints(pcl1_in_world_frame_with_noise_,
                                  ref_pose_.inverse(),
                                  pcl1_in_ref_frame_with_noise_);
  icp_cov::utils::TransformPoints(pcl2_in_world_frame_with_noise_,
                                  ref_pose_.inverse(),
                                  pcl2_in_ref_frame_with_noise_);

  // step 7: generate init pose + add noise
  icp_transform_ = target_pose2_ * target_pose1_.inverse();
  AddNoiseToIcpTransform();
}

void DataGenerator::Generate2d() {
  auto config = icp_cov::Config::Instance();
  const Eigen::Vector3d target_size(config->kTargetSize);
  const double angle_resolution =
      config->kLaserHorizontalAngleResolution;  // unit: degree
  //   const bool output_points_in_world_frame = true;
  GeneratePoints(ego_pose1_, target_pose1_, target_size, angle_resolution,
                 pcl1_in_ego_frame_);
  GeneratePoints(ego_pose2_, target_pose2_, target_size, angle_resolution,
                 pcl2_in_ego_frame_);
}

void DataGenerator::Generate3d() {
  GeneratePointsInEgoFrame3dVersion(ego_pose1_, target_pose1_,
                                    pcl1_in_ego_frame_);
  GeneratePointsInEgoFrame3dVersion(ego_pose2_, target_pose2_,
                                    pcl2_in_ego_frame_);
}

void DataGenerator::GeneratePointsInEgoFrame3dVersion(
    const Eigen::Affine3d& ego_pose, const Eigen::Affine3d& target_pose,
    std::vector<Eigen::Vector3d>& pcl_in_ego_frame) {
  auto config = icp_cov::Config::Instance();
  // step 1: calculate target_center_pose_in_lidar_frame
  const Eigen::Affine3d lidar_pose_in_ego_frame = config->kLidarPoseInEgoFrame;
  const Eigen::Vector3d target_size = config->kTargetSize;
  const Eigen::Affine3d lidar_pose_in_world_frame =
      ego_pose * lidar_pose_in_ego_frame;
  const Eigen::Vector3d target_center_in_target_frame(0.5 * target_size(0), 0,
                                                      -0.5 * target_size(2));
  // target pose is pose of rear wheel center
  const Eigen::Affine3d target_center_pose = icp_cov::utils::RtToAffine3d(
      target_pose.rotation(), target_pose * target_center_in_target_frame);
  const Eigen::Affine3d target_center_pose_in_lidar_frame =
      lidar_pose_in_world_frame.inverse() * target_center_pose;
  // step 2: generate visible planes (represented in sensor frame)
  std::vector<FiniteRectangle> visible_planes;
  icp_cov::utils::CalculateVisiblePlanesOfTargetToSensor(
      ego_pose, target_pose, lidar_pose_in_ego_frame, target_size,
      visible_planes);
  // step 3: generate points from intersection of laser rays and visible planes
  const double horizontal_angle_resolution =
      config->kLaserHorizontalAngleResolution;
  const double vertical_angle_resolution =
      config->kLaserVerticalAngleResolution;
  const int laser_number = config->kLaserNumber;
  const int horizontal_laser_index = config->kHorizontalLaserIndex;
  assert(horizontal_angle_resolution > 1.0e-6);
  assert(vertical_angle_resolution > 1.0e-6);
  assert(laser_number > 0);
  const int point_number_per_laser = 360 / horizontal_angle_resolution;
  static std::vector<Eigen::Vector3d> pcl_in_lidar_frame;
  pcl_in_lidar_frame.clear();
  int hindex_begin = -1;
  int hindex_end = -1;
  int vindex_begin = -1;
  int vindex_end = -1;
  CalculateHIndexAndVIndexRange(target_center_pose_in_lidar_frame, target_size,
                                hindex_begin, hindex_end, vindex_begin,
                                vindex_end);
  for (int hindex = hindex_begin; hindex < hindex_end; ++hindex) {
    double hangle = hindex * horizontal_angle_resolution * config->kDegToRad;
    for (int vindex = vindex_begin; vindex < vindex_end; ++vindex) {
      double vangle = (vindex - horizontal_laser_index) *
                      vertical_angle_resolution * config->kDegToRad;
      // generate one point from intersection of laser ray and visible plane
      Eigen::Vector3d generated_point;
      if (GenerateOnePointFromHangleAndVangle(hangle, vangle, visible_planes,
                                              generated_point)) {
        pcl_in_lidar_frame.emplace_back(generated_point);
      }
    }
  }
  // step 4: transform points to ego frame
  icp_cov::utils::TransformPoints(pcl_in_lidar_frame, lidar_pose_in_ego_frame,
                                  pcl_in_ego_frame);
}

void DataGenerator::CalculateHIndexAndVIndexRange(
    const Eigen::Affine3d& target_center_pose_in_lidar_frame,
    const Eigen::Vector3d& target_size, int& hindex_begin, int& hindex_end,
    int& vindex_begin, int& vindex_end) {
  // step 1: calculate eight corner points of target in lidar frame
  const std::vector<Eigen::Vector3d>
      eight_corner_points_in_target_center_frame = {
          Eigen::Vector3d(-0.5 * target_size(0), -0.5 * target_size(1),
                          0.5 * target_size(2)),
          Eigen::Vector3d(0.5 * target_size(0), -0.5 * target_size(1),
                          0.5 * target_size(2)),
          Eigen::Vector3d(0.5 * target_size(0), 0.5 * target_size(1),
                          0.5 * target_size(2)),
          Eigen::Vector3d(-0.5 * target_size(0), 0.5 * target_size(1),
                          0.5 * target_size(2)),
          Eigen::Vector3d(-0.5 * target_size(0), -0.5 * target_size(1),
                          -0.5 * target_size(2)),
          Eigen::Vector3d(0.5 * target_size(0), -0.5 * target_size(1),
                          -0.5 * target_size(2)),
          Eigen::Vector3d(0.5 * target_size(0), 0.5 * target_size(1),
                          -0.5 * target_size(2)),
          Eigen::Vector3d(-0.5 * target_size(0), 0.5 * target_size(1),
                          -0.5 * target_size(2)),
      };
  std::vector<Eigen::Vector3d> eight_corner_points_in_lidar_frame;
  icp_cov::utils::TransformPoints(eight_corner_points_in_target_center_frame,
                                  target_center_pose_in_lidar_frame,
                                  eight_corner_points_in_lidar_frame);
  // step 2: calculate hindex and vindex of eight corner points and then derive
  // the index range
  auto config = icp_cov::Config::Instance();
  const int laser_number = config->kLaserNumber;
  const int horizontal_laser_index = config->kHorizontalLaserIndex;
  const double horizontal_angle_resolution =
      config->kLaserHorizontalAngleResolution;
  const double vertical_angle_resolution =
      config->kLaserVerticalAngleResolution;
  // step 2.0: set initial value for hindex_begin, hindex_end, vindex_begin,
  // vindex_end
  hindex_begin = 360 / horizontal_angle_resolution;
  hindex_end = 0;
  vindex_begin = laser_number;
  vindex_end = 0;
  for (const Eigen::Vector3d& corner_point :
       eight_corner_points_in_lidar_frame) {
    // step 2.1: calculate hangle and vangle
    const Eigen::Vector3d normalized_corner_point = corner_point.normalized();
    const double vangle = -std::asin(normalized_corner_point(2)) *
                          icp_cov::Config::Instance()->kRadToDeg;
    // assert(vangle > -horizontal_laser_index * vertical_angle_resolution);
    // assert(vangle <
    //        (laser_number - horizontal_laser_index) * vertical_angle_resolution);
    double hangle =
        std::atan2(normalized_corner_point(1), normalized_corner_point(0)) *
        icp_cov::Config::Instance()->kRadToDeg;
    hangle = hangle > 0 ? hangle : hangle + 360;
    assert(hangle >= 0);
    assert(hangle <= 360);
    // step 2.2: calculate hindex and vindex
    const double hindex = hangle / horizontal_angle_resolution;
    const double vindex = std::min(static_cast<double>(laser_number), 
        std::max(0.0, vangle / vertical_angle_resolution + horizontal_laser_index));
    // step 2.3: calculate range
    hindex_begin =
        hindex_begin > std::floor(hindex) ? std::floor(hindex) : hindex_begin;
    hindex_end =
        hindex_end < std::ceil(hindex) ? std::ceil(hindex) : hindex_end;
    vindex_begin =
        vindex_begin > std::floor(vindex) ? std::floor(vindex) : vindex_begin;
    vindex_end =
        vindex_end < std::ceil(vindex) ? std::ceil(vindex) : vindex_end;
  }
  assert(hindex_begin >= 0 && hindex_begin < hindex_end &&
         hindex_end <= 360 / horizontal_angle_resolution);
  assert(vindex_begin >= 0 && vindex_begin < vindex_end &&
         vindex_end <= laser_number);
}

bool DataGenerator::GenerateOnePointFromHangleAndVangle(
    const double hangle, const double vangle,
    const std::vector<FiniteRectangle>& visible_planes,
    Eigen::Vector3d& generated_point) {
  bool success = false;
  double min_d = std::numeric_limits<double>::max();
  for (auto& visible_plane : visible_planes) {
    // step 1: calculate d of generated_point
    Eigen::Vector3d intersection_point;
    const bool point_is_valid =
        visible_plane.CalculateIntersectionPointWithOneRay(hangle, vangle,
                                                           intersection_point);
    if (!point_is_valid) continue;
    // step 2: post process
    success = true;
    // select the closest intersection point as lidar observation
    if (intersection_point.norm() < min_d) {
      generated_point = intersection_point;
      min_d = intersection_point.norm();
    }
  }
  return success;
}

void DataGenerator::GeneratePoints(
    const Eigen::Affine3d& ego_pose, const Eigen::Affine3d& target_pose,
    const Eigen::Vector3d& target_size, const double angle_resolution,
    std::vector<Eigen::Vector3d>& pcl_in_ego_frame) {
  // step 1: generate line segment endpoints
  // step 1.1: generate line segment endpoints in target frame
  const Eigen::Vector3d ego_position = ego_pose.translation();
  const Eigen::Vector3d target_position = target_pose.translation();
  Eigen::Vector3d end_point1_in_target_frame(
      target_position(0) > ego_position(0) ? target_size(0) : -target_size(0),
      0, 0);  // point in x axis
  Eigen::Vector3d end_point2_in_target_frame(
      0, 0, 0);  // anchor point: the closest point in the target
  Eigen::Vector3d end_point3_in_target_frame(
      0,
      target_position(1) > ego_position(1) ? target_size(1) : -target_size(1),
      0);  // point in y axis
  // step 1.2: generate line segment endpoints in world frame
  Eigen::Vector3d end_point1_in_world_frame =
      target_pose * end_point1_in_target_frame;
  Eigen::Vector3d end_point2_in_world_frame =
      target_pose * end_point2_in_target_frame;
  Eigen::Vector3d end_point3_in_world_frame =
      target_pose * end_point3_in_target_frame;
  // step 1.3: generate line segment endpoints in ego frame
  Eigen::Vector3d end_point1_in_ego_frame =
      ego_pose.inverse() * end_point1_in_world_frame;
  Eigen::Vector3d end_point2_in_ego_frame =
      ego_pose.inverse() * end_point2_in_world_frame;
  Eigen::Vector3d end_point3_in_ego_frame =
      ego_pose.inverse() * end_point3_in_world_frame;

  // step 2: generate points in ego frame by calculating intersection points of
  // laser ray and line segments
  // step 2.1: generate points in x axis of target
  std::vector<Eigen::Vector3d> target_x_axis_laser_points_in_ego_frame;
  GeneratePointsOnLineSegmentInEgoFrame(
      end_point1_in_ego_frame, end_point2_in_ego_frame, angle_resolution,
      target_x_axis_laser_points_in_ego_frame);
  // step 2.2: generate points in y axis of target
  std::vector<Eigen::Vector3d> target_y_axis_laser_points_in_ego_frame;
  GeneratePointsOnLineSegmentInEgoFrame(
      end_point2_in_ego_frame, end_point3_in_ego_frame, angle_resolution,
      target_y_axis_laser_points_in_ego_frame);

  // step 3: save points
  pcl_in_ego_frame.clear();
  pcl_in_ego_frame.reserve(target_x_axis_laser_points_in_ego_frame.size() +
                           target_y_axis_laser_points_in_ego_frame.size());
  std::copy(target_x_axis_laser_points_in_ego_frame.begin(),
            target_x_axis_laser_points_in_ego_frame.end(),
            std::back_inserter(pcl_in_ego_frame));
  std::copy(target_y_axis_laser_points_in_ego_frame.begin(),
            target_y_axis_laser_points_in_ego_frame.end(),
            std::back_inserter(pcl_in_ego_frame));
}

/**
 * point1, point2, generated_points are all in ego frame
 */
void DataGenerator::GeneratePointsOnLineSegmentInEgoFrame(
    const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
    const double angle_resolution,
    std::vector<Eigen::Vector3d>& generated_points) {
  assert(angle_resolution > 1.0e-6);
  generated_points.clear();
  // step 1: calculate angle range
  const double angle1 = std::atan2(point1(1), point1(0)) * 180 / M_PI;
  const double angle2 = std::atan2(point2(1), point2(0)) * 180 / M_PI;
  const double begin_angle = std::min(angle1, angle2);
  const double end_angle = std::max(angle1, angle2);
  const int begin_angle_index = std::ceil(begin_angle / angle_resolution);
  const int end_angle_index = std::ceil(end_angle / angle_resolution);
  generated_points.reserve(end_angle_index - begin_angle_index);
  // step 2: generate points
  // step 2.1: calculate line equation
  const Eigen::Vector3d line_segment =
      Eigen::Vector3d(point1(0), point1(1), 1)
          .cross(Eigen::Vector3d(point2(0), point2(1), 1));
  for (int i = begin_angle_index; i < end_angle_index; ++i) {
    // step 2.2: calculate laser ray equation
    const double angle = i * angle_resolution * M_PI / 180.0;
    const Eigen::Vector3d laser_ray = Eigen::Vector3d(0, 0, 1).cross(
        Eigen::Vector3d(cos(angle), sin(angle), 1));
    // step 2.3: calculate intersection point
    const Eigen::Vector3d intersection_point = line_segment.cross(laser_ray);
    assert(std::fabs(intersection_point(2)) > 1.0e-6);
    generated_points.emplace_back(intersection_point(0) / intersection_point(2),
                                  intersection_point(1) / intersection_point(2),
                                  0);
  }
}
// TODO(clssucceed@gmail.com): laser point noise is one-dimensional(along the
// laser ray), not multi-dimensional
void DataGenerator::AddNoiseToPoints(
    const std::vector<Eigen::Vector3d>& pcl_in_ego_frame,
    std::vector<Eigen::Vector3d>& pcl_in_ego_frame_with_noise) {
  pcl_in_ego_frame_with_noise.clear();
  pcl_in_ego_frame_with_noise.reserve(pcl_in_ego_frame.size());
  constexpr double kNoiseSigma = 0.03;  // unit: m
  for (const auto& point : pcl_in_ego_frame) {
    pcl_in_ego_frame_with_noise.emplace_back(
        point + icp_cov::utils::PointNoise2d(kNoiseSigma));
  }
}

void DataGenerator::AddNoiseToIcpTransform() {
  icp_transform_init_ =
      icp_transform_ *
      icp_cov::utils::TransformNoise(2.0 / 180 * M_PI, 0.2);  // 2deg + 0.2m
}
}  // namespace icp_cov