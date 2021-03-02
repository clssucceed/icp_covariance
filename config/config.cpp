#include "config.h"

#include <iostream>

#include "../utils.h"

namespace icp_cov {
Config* Config::config_ = nullptr;

const Config* Config::Instance() {
  if (nullptr == config_) {
    config_ = new Config();
  }

  return config_;
}

using namespace icp_cov::utils;
Config::Config() {
  yaml_node_ = YAML::LoadFile("../config/config.yaml");
  // ego pose
  kEgoPose1InWorldFrame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["ego_pose1_in_world_frame"]["yaw"].as<double>(),
          yaml_node_["ego_pose1_in_world_frame"]["pitch"].as<double>(),
          yaml_node_["ego_pose1_in_world_frame"]["roll"].as<double>())),
      Eigen::Vector3d(
          yaml_node_["ego_pose1_in_world_frame"]["x"].as<double>(),
          yaml_node_["ego_pose1_in_world_frame"]["y"].as<double>(),
          yaml_node_["ego_pose1_in_world_frame"]["z"].as<double>()));
  std::cout << "ego_pose1_in_world_frame: " << std::endl
            << kEgoPose1InWorldFrame.matrix() << std::endl;
  kEgoPose2InWorldFrame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["ego_pose2_in_world_frame"]["yaw"].as<double>(),
          yaml_node_["ego_pose2_in_world_frame"]["pitch"].as<double>(),
          yaml_node_["ego_pose2_in_world_frame"]["roll"].as<double>())),
      Eigen::Vector3d(
          yaml_node_["ego_pose2_in_world_frame"]["x"].as<double>(),
          yaml_node_["ego_pose2_in_world_frame"]["y"].as<double>(),
          yaml_node_["ego_pose2_in_world_frame"]["z"].as<double>()));
  std::cout << "ego_pose2_in_world_frame: " << std::endl
            << kEgoPose2InWorldFrame.matrix() << std::endl;
  // target pose
  kTargetPose1InEgo1Frame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["target_pose1_in_ego1_frame"]["yaw"].as<double>(),
          yaml_node_["target_pose1_in_ego1_frame"]["pitch"].as<double>(),
          yaml_node_["target_pose1_in_ego1_frame"]["roll"].as<double>())),
      Eigen::Vector3d(
          yaml_node_["target_pose1_in_ego1_frame"]["x"].as<double>(),
          yaml_node_["target_pose1_in_ego1_frame"]["y"].as<double>(),
          yaml_node_["target_pose1_in_ego1_frame"]["z"].as<double>()));
  std::cout << "target_pose1_in_ego1_frame: " << std::endl
            << kTargetPose1InEgo1Frame.matrix() << std::endl;
  kTargetPose2InEgo2Frame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["target_pose2_in_ego2_frame"]["yaw"].as<double>(),
          yaml_node_["target_pose2_in_ego2_frame"]["pitch"].as<double>(),
          yaml_node_["target_pose2_in_ego2_frame"]["roll"].as<double>())),
      Eigen::Vector3d(
          yaml_node_["target_pose2_in_ego2_frame"]["x"].as<double>(),
          yaml_node_["target_pose2_in_ego2_frame"]["y"].as<double>(),
          yaml_node_["target_pose2_in_ego2_frame"]["z"].as<double>()));
  std::cout << "target_pose2_in_ego2_frame: " << std::endl
            << kTargetPose2InEgo2Frame.matrix() << std::endl;
  kTargetSize =
      Eigen::Vector3d(yaml_node_["target_size"]["length"].as<double>(),
                      yaml_node_["target_size"]["width"].as<double>(),
                      yaml_node_["target_size"]["height"].as<double>());
  std::cout << "target_size: " << kTargetSize.transpose() << std::endl;
  kCellSizeOnTarget = yaml_node_["cell_size_on_target"].as<double>();
  std::cout << "cell_size_on_target: " << kCellSizeOnTarget << std::endl;
  // laser config
  kLaserHorizontalAngleResolution =
      yaml_node_["horizontal_angle_resolution"].as<double>();
  std::cout << "horizontal_angle_resolution: "
            << kLaserHorizontalAngleResolution << std::endl;
  kLaserVerticalAngleResolution =
      yaml_node_["vertical_angle_resolution"].as<double>();
  std::cout << "vertical_angle_resolution: " << kLaserVerticalAngleResolution
            << std::endl;
  kLaserNumber = yaml_node_["laser_number"].as<int>();
  std::cout << "laser_number: " << kLaserNumber << std::endl;
  kHorizontalLaserIndex = yaml_node_["horizontal_laser_index"].as<int>();
  std::cout << "horizontal_laser_index: " << kHorizontalLaserIndex << std::endl;
  kLidarPoseInEgoFrame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["lidar_pose_in_ego_frame"]["yaw"].as<double>(),
          yaml_node_["lidar_pose_in_ego_frame"]["pitch"].as<double>(),
          yaml_node_["lidar_pose_in_ego_frame"]["roll"].as<double>())),
      Eigen::Vector3d(yaml_node_["lidar_pose_in_ego_frame"]["x"].as<double>(),
                      yaml_node_["lidar_pose_in_ego_frame"]["y"].as<double>(),
                      yaml_node_["lidar_pose_in_ego_frame"]["z"].as<double>()));
  std::cout << "lidar_pose_in_ego_frame: " << std::endl
            << kLidarPoseInEgoFrame.matrix() << std::endl;

  // camera config
  kCameraPoseInEgoFrame = RtToAffine3d(
      ypr2R(Eigen::Vector3d(
          yaml_node_["camera_pose_in_ego_frame"]["yaw"].as<double>(),
          yaml_node_["camera_pose_in_ego_frame"]["pitch"].as<double>(),
          yaml_node_["camera_pose_in_ego_frame"]["roll"].as<double>())),
      Eigen::Vector3d(
          yaml_node_["camera_pose_in_ego_frame"]["x"].as<double>(),
          yaml_node_["camera_pose_in_ego_frame"]["y"].as<double>(),
          yaml_node_["camera_pose_in_ego_frame"]["z"].as<double>()));
  std::cout << "camera_pose_in_ego_frame: " << std::endl
            << kCameraPoseInEgoFrame.matrix() << std::endl;
  kCameraIntrinsicMatrix
      << yaml_node_["camera_intrinsic_matrix"]["fx"].as<double>(),
      0, yaml_node_["camera_intrinsic_matrix"]["cx"].as<double>(), 0,
      yaml_node_["camera_intrinsic_matrix"]["fy"].as<double>(),
      yaml_node_["camera_intrinsic_matrix"]["cy"].as<double>(), 0, 0, 1;
  std::cout << "camera_intrinsic_matrix: " << std::endl
            << kCameraIntrinsicMatrix << std::endl;
  kImageWidth = yaml_node_["image_width"].as<int>();
  std::cout << "image_width: " << kImageWidth << std::endl;
  kImageHeight = yaml_node_["image_height"].as<int>();
  std::cout << "image_height: " << kImageHeight << std::endl;

  // noise
  kLidarPclNoise = yaml_node_["lidar_pcl_noise"].as<double>();
  std::cout << "lidar_pcl_noise: " << kLidarPclNoise << std::endl;
  kInitialRotationNoise = yaml_node_["initial_rotation_noise"].as<double>();
  std::cout << "initial_rotation_noise: " << kInitialRotationNoise << std::endl;
  kInitialTranslationNoise = yaml_node_["initial_translation_noise"].as<double>();
  std::cout << "initial_translation_noise: " << kInitialTranslationNoise << std::endl;

  // others
  kDeltaTimeBetweenTwoFrame =
      yaml_node_["delta_time_between_two_frame"].as<double>();
  std::cout << "delta_time_between_two_frame: " << kDeltaTimeBetweenTwoFrame
            << std::endl;
  kGenerate3d = yaml_node_["generate_3d_data"].as<bool>();
  std::cout << "generate_3d_data: " << static_cast<int>(kGenerate3d)
            << std::endl;
}
}  // namespace icp_cov