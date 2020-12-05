#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
class DataGenerator {
 public:
  static DataGenerator* Instance();

 public:
  const std::vector<Eigen::Vector3d>& pcl1_in_ego_frame() {
    return pcl1_in_ego_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_world_frame() {
    return pcl1_in_world_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ego_frame() {
    return pcl2_in_ego_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_world_frame() {
    return pcl2_in_world_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_ego_frame_with_noise() {
    return pcl1_in_ego_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_world_frame_with_noise() {
    return pcl1_in_world_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ego_frame_with_noise() {
    return pcl2_in_ego_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_world_frame_with_noise() {
    return pcl2_in_world_frame_with_noise_;
  }

 private:
  static DataGenerator* data_generator_;

 private:
  DataGenerator() {
    Generate();
  }

  void Generate();
  void GeneratePoints(const Eigen::Affine3d& ego_pose,
                      const Eigen::Affine3d& target_pose,
                      const Eigen::Vector3d& target_size,
                      const double angle_resolution,
                      std::vector<Eigen::Vector3d>& pcl_in_ego_frame,
                      std::vector<Eigen::Vector3d>& pcl_in_world_frame);
  void GeneratePointsOnLineSegmentInEgoFrame(
      const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
      const double angle_resolution,
      std::vector<Eigen::Vector3d>& generated_points);

 private:
  // ground truth
  std::vector<Eigen::Vector3d> pcl1_in_ego_frame_;
  std::vector<Eigen::Vector3d> pcl2_in_ego_frame_;
  std::vector<Eigen::Vector3d> pcl1_in_world_frame_;
  std::vector<Eigen::Vector3d> pcl2_in_world_frame_;
  Eigen::Affine3d icp_transform_;

  // with noise
  std::vector<Eigen::Vector3d> pcl1_in_ego_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl2_in_ego_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl1_in_world_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl2_in_world_frame_with_noise_;
  Eigen::Affine3d icp_transform_init_;
};
}  // namespace icp_cov