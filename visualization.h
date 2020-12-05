#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

namespace icp_cov {
namespace visualization {
void DrawPoints(const std::vector<Eigen::Vector3d>& points, cv::Mat& canvas);
void GenerateCanvas(cv::Mat& canvas);
void Show(const cv::Mat& canvas);
cv::Point2d CoordinateTransformation(const Eigen::Vector3d& point);
}  // namespace visualization
}  // namespace icp_cov