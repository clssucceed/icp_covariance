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
  std::cout << kTargetSize << std::endl;
  kLaserHorizontalAngleResolution =
      yaml_node_["horizontal_angle_resolution"].as<double>();
  std::cout << "horizontal_angle_resolution: "
            << kLaserHorizontalAngleResolution << std::endl;
  kDeltaTimeBetweenTwoFrame =
      yaml_node_["delta_time_between_two_frame"].as<double>();
  std::cout << "delta_time_between_two_frame: " << kDeltaTimeBetweenTwoFrame
            << std::endl;
}
}  // namespace icp_cov