#include "visualization.h"

namespace icp_cov {
namespace visualization {

const cv::Scalar kColorRed{0, 0, 255};
const double resolution = 0.02;  // unit: m/pixel
const double width_range = 20;   // unit: m
const double height_range = 20;  // unit: m
const int image_width = width_range * 2 / resolution;
const int image_height = height_range * 2 / resolution;
const cv::Point2d canvas_center(image_width * 0.5, image_height * 0.5);

void DrawPoints(const std::vector<Eigen::Vector3d>& points, cv::Mat& canvas) {
  const double point_radius = 3;
  const cv::Scalar point_color = kColorRed;
  for (const auto& point : points) {
    cv::circle(canvas, CoordinateTransformation(point), point_radius,
               point_color, -1);
  }
}

void GenerateCanvas(cv::Mat& canvas) {
  canvas = cv::Mat::zeros(image_height, image_width, CV_8UC3);
  cv::circle(canvas, canvas_center, 10, kColorRed, -1);
  const cv::Point2d start_point_on_horizontal_line(0, image_height * 0.5);
  const cv::Point2d end_point_on_horizontal_line(image_width,
                                                 image_height * 0.5);
  cv::line(canvas, start_point_on_horizontal_line, end_point_on_horizontal_line,
           kColorRed, 5);
  const cv::Point2d start_point_on_vertical_line(image_width * 0.5, 0);
  const cv::Point2d end_point_on_vertical_line(image_width * 0.5, image_height);
  cv::line(canvas, start_point_on_vertical_line, end_point_on_vertical_line,
           kColorRed, 5);
}

void Show(const cv::Mat& canvas) {
  const cv::Mat& to_be_showed =
      canvas(cv::Range(0, canvas.rows), cv::Range(0, canvas.cols));
  const std::string window_name = "icp_cov";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, to_be_showed);
  cv::waitKey(0);
}

cv::Point2d CoordinateTransformation(const Eigen::Vector3d& point) {
  return cv::Point2d(point(1), -point(0)) / resolution + canvas_center;
}
}  // namespace visualization
}  // namespace icp_cov