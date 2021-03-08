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
  // ego config
  Eigen::Affine3d kEgoPose1InWorldFrame;
  Eigen::Affine3d kEgoPose2InWorldFrame;
  // target config
  Eigen::Affine3d kTargetPose1InEgo1Frame;
  Eigen::Affine3d kTargetPose2InEgo2Frame;
  Eigen::Vector3d kTargetSize;
  double kCellSizeOnTarget;
  // lidar config
  double kLaserHorizontalAngleResolution;  // unit: degree
  double kLaserVerticalAngleResolution;    // unit: degree
  int kLaserNumber;
  int kHorizontalLaserIndex;
  Eigen::Affine3d kLidarPoseInEgoFrame;
  // camera config
  Eigen::Affine3d kCameraPoseInEgoFrame;
  Eigen::Matrix3d kCameraIntrinsicMatrix;
  int kImageWidth;
  int kImageHeight;
  // noise
  double kLidarPclNoise;
  double kInitialRotationNoise;
  double kInitialTranslationNoise;
  // downsample
  bool kDownsample;
  double kLeafSize;
  // edge detection
  bool kEdgeDetection;
  double kRadiusRatio;
  double kMevrThRatio;
  double kMevrThLowBound;
  int kMevrSelectNumUpBound;
  double kNnnThRatio;
  int kAllSelectNumUpBound;
  // others
  double kDeltaTimeBetweenTwoFrame;  // unit: sec
  bool kGenerate3d;
  bool kPclAlignmentVisualization;

 private:
  static Config* config_;
  Config();

 private:
  YAML::Node yaml_node_;
};
}  // namespace icp_cov