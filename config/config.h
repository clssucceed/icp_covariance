#pragma once
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

namespace icp_cov {
class Config {
 public:
  // NOTE: const is to ensure parameters can not be modified externally
  static const Config* Instance();

 public:
  // used to get non-frequently used parameters externally
  const YAML::Node& yaml_node() const { return yaml_node_; }

 public:
  // const
  static constexpr double kRadToDeg = 180.0 / M_PI;
  static constexpr double kDegToRad = 1.0 / kRadToDeg;

  // frequently used parameters
  Eigen::Affine3d kEgoPose1InWorldFrame;
  Eigen::Affine3d kEgoPose2InWorldFrame;
  Eigen::Affine3d kTargetPose1InEgo1Frame;
  Eigen::Affine3d kTargetPose2InEgo2Frame;
  Eigen::Vector3d kTargetSize;
  double kLaserHorizontalAngleResolution;  // unit: degree
  double kLaserVerticalAngleResolution;    // unit: degree
  int kLaserNumber;
  int kHorizontalLaserIndex;
  Eigen::Affine3d kLidarPoseInEgoFrame;
  double kDeltaTimeBetweenTwoFrame;  // unit: sec
  bool kGenerate3d;

 private:
  static Config* config_;
  Config();

 private:
  YAML::Node yaml_node_;
};
}  // namespace icp_cov