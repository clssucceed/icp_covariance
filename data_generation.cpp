#include "data_generation.h"

#include "utils.h"
#include "visualization.h"

namespace icp_cov {
namespace data_gen {

void DataGeneration(std::vector<Eigen::Vector3d>& pcl1,
                    std::vector<Eigen::Vector3d>& pcl2,
                    Eigen::Affine3d& init_pose) {
  // TODO(clssucceed@gmail.com): pose1和world系不重合

  // step 1: set ego pose
  Eigen::Affine3d ego_pose1 = icp_cov::utils::RtToAffine3d(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  Eigen::Affine3d ego_pose2 = icp_cov::utils::RtToAffine3d(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // step 2: set target pose
  Eigen::Affine3d target_pose1 = icp_cov::utils::RtToAffine3d(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(10, 5, 0));
  Eigen::Affine3d target_pose2 = icp_cov::utils::RtToAffine3d(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(11, 5, 0));

  // step 3: generate points
  const Eigen::Vector3d target_size(4, 2, 0);
  const double angle_resolution = 0.2;  // unit: degree
  const bool output_points_in_world_frame = true;
  GeneratePoints(ego_pose1, target_pose1, target_size, angle_resolution,
                 output_points_in_world_frame, pcl1);
  GeneratePoints(ego_pose2, target_pose2, target_size, angle_resolution,
                 output_points_in_world_frame, pcl2);

  // step 4: generate init pose
  init_pose = target_pose2 * target_pose1.inverse();

  // step 5: visualization
  cv::Mat canvas;
  icp_cov::visualization::GenerateCanvas(canvas);
  icp_cov::visualization::DrawPoints(pcl1, canvas);
  icp_cov::visualization::Show(canvas);
}

void GeneratePoints(const Eigen::Affine3d& ego_pose,
                    const Eigen::Affine3d& target_pose,
                    const Eigen::Vector3d& target_size,
                    const double angle_resolution,
                    const bool output_points_in_world_frame,
                    std::vector<Eigen::Vector3d>& pcl) {
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

  // step 3: transform points to world frame
  pcl.clear();
  pcl.reserve(target_x_axis_laser_points_in_ego_frame.size() +
              target_y_axis_laser_points_in_ego_frame.size());
  if (output_points_in_world_frame) {
    icp_cov::utils::TransformPoints(target_x_axis_laser_points_in_ego_frame,
                                    ego_pose, pcl);
    icp_cov::utils::TransformPoints(target_y_axis_laser_points_in_ego_frame,
                                    ego_pose, pcl);
  } else {
    std::copy(target_x_axis_laser_points_in_ego_frame.begin(),
              target_x_axis_laser_points_in_ego_frame.end(),
              std::back_inserter(pcl));
    std::copy(target_y_axis_laser_points_in_ego_frame.begin(),
              target_y_axis_laser_points_in_ego_frame.end(),
              std::back_inserter(pcl));
  }
}

/**
 * point1, point2, generated_points are all in ego frame
 */
void GeneratePointsOnLineSegmentInEgoFrame(
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
}  // namespace data_gen
}  // namespace icp_cov