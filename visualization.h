#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "config.h"

namespace icp_cov {
class Visualization {
 public:
  static Visualization* Instance();
  void DrawPoints(const std::vector<Eigen::Vector3d>& points,
                  const cv::Scalar& point_color);
  void Show();
  void Save(const std::string& name);
  void ResetCanvas() { GenerateCanvas(); }

 private:
  // const
  static constexpr double kResolution = kPixelResolution;  // unit: m/pixel
  static constexpr double kWidthRange = kEgoYRange;        // unit: m
  static constexpr double kHeightRange = kEgoXRange;       // unit: m
  static constexpr int kImageWidth = kWidthRange * 2 / kResolution;
  static constexpr int kImageHeight = kHeightRange * 2 / kResolution;
  const cv::Point2d kCanvasCenter;

  Visualization() : kCanvasCenter(kImageWidth * 0.5, kImageHeight * 0.5) {
    GenerateCanvas();
  }
  static Visualization* visualization_;
  cv::Mat canvas_;

  void GenerateCanvas();
  cv::Point2d CoordinateTransformation(const Eigen::Vector3d& point);
};
}  // namespace icp_cov