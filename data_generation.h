#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
namespace data_gen {
void DataGeneration(std::vector<Eigen::Vector3d>& pcl1,
                    std::vector<Eigen::Vector3d>& pcl2,
                    Eigen::Affine3d& init_pose);
void GeneratePoints(const Eigen::Affine3d& ego_pose,
                    const Eigen::Affine3d& target_pose,
                    const Eigen::Vector3d& target_size,
                    const double angle_resolution,
                    const bool output_points_in_world_frame,
                    std::vector<Eigen::Vector3d>& pcl);
void GeneratePointsOnLineSegmentInEgoFrame(
    const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
    const double angle_resolution,
    std::vector<Eigen::Vector3d>& generated_points);
}  // namespace data_gen
}  // namespace icp_cov