#include "visualization.h"

#include "color.h"

namespace icp_cov {

Visualization* Visualization::visualization_ = nullptr;

Visualization* Visualization::Instance() {
  if (visualization_ == nullptr) {
    visualization_ = new Visualization();
  }
  return visualization_;
}

void Visualization::DrawPoints(const std::vector<Eigen::Vector3d>& points,
                               const cv::Scalar& point_color) {
  const double kPointRadius = 1;
  for (const auto& point : points) {
    cv::circle(canvas_, CoordinateTransformation(point), kPointRadius,
               point_color, -1);
  }
}

void Visualization::GenerateCanvas() {
  canvas_ = cv::Mat::zeros(kImageHeight, kImageWidth, CV_8UC3);
  cv::circle(canvas_, kCanvasCenter, 10, kColorRed, -1);
  const cv::Point2d start_point_on_horizontal_line(0, kImageHeight * 0.5);
  const cv::Point2d end_point_on_horizontal_line(kImageWidth,
                                                 kImageHeight * 0.5);
  cv::line(canvas_, start_point_on_horizontal_line,
           end_point_on_horizontal_line, kColorWhite, 5);
  const cv::Point2d start_point_on_vertical_line(kImageWidth * 0.5, 0);
  const cv::Point2d end_point_on_vertical_line(kImageWidth * 0.5, kImageHeight);
  cv::line(canvas_, start_point_on_vertical_line, end_point_on_vertical_line,
           kColorWhite, 5);
}

void Visualization::Show() {
  const cv::Mat& to_be_showed =
      canvas_(cv::Range(0, canvas_.rows), cv::Range(0, canvas_.cols));
  const std::string window_name = "icp_cov";
  cv::imwrite(window_name + ".png", to_be_showed);
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, to_be_showed);
  cv::waitKey(0);
}

cv::Point2d Visualization::CoordinateTransformation(
    const Eigen::Vector3d& point) {
  return cv::Point2d(point(1), -point(0)) / kResolution + kCanvasCenter;
}
}  // namespace icp_cov