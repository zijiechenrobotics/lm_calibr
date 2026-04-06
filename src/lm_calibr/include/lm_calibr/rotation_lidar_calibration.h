#pragma once

#include <glog/logging.h>
#include <pcl/common/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tbb/global_control.h>
#include <tbb/parallel_reduce.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <filesystem>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/so3.hpp>
#include <unordered_map>
#include <vector>

#include "lm_calibr/adaptive_voxel_map.h"
#include "lm_calibr/concurrent_voxel_grid.hpp"

constexpr double TWO_PI = 2.0 * M_PI;

struct EncoderMsg {
  double angle = 0.0;  // 0~2pi
  double angle_vel = 0.0;
  double timestamp = 0.0;
};

struct CloudMsg {
  pcl::PointCloud<PointType>::Ptr cloud_ptr;
  double timestamp = 0.0;
};

enum LidarType { SIM = 0, LIVOX };

class RotationLidarCalibration {
 public:
  struct Config {
    size_t max_iter = 50;
    float max_voxel_size = 1.0;
    std::vector<double> eigen_threshold = {0.1, 0.1, 0.1};
    size_t max_layer = 2;
    double downsample_size = 0.1;

    // minimum range of the laser point
    double min_range = 1.0;
    // maximum range of the laser point
    double max_range = 50.0;

    // 0:Omni LiDAR; 1:Non-Omni LiDAR
    int DH_type = 0;

    double outlier_remove_ratio = 0.05;

    // skip the first few seconds of data to remove dynamic objects
    double rosbag_skip = 0.0;
    // If accumulated angle exceeds threshold
    // discard remaining data to prevent excessive point cloud
    double angle_threshold = 4.0 * M_PI;

    void Print() {
      std::cout << "[RotationLidarCalibration::Config] config param:"
                << std::endl
                << "\tmax_iter: " << max_iter << std::endl
                << "\tmax_voxel_size: " << max_voxel_size << std::endl
                << "\tmax_layer: " << max_layer << std::endl
                << "\tdownsample_size: " << downsample_size << std::endl
                << "\trosbag_skip: " << rosbag_skip << std::endl
                << "\tangle_threshold: " << angle_threshold << std::endl;

      std::cout << "\teigen_threshold: ";
      for (const auto thr : eigen_threshold) {
        std::cout << thr << ", ";
      }
      std::cout << std::endl;
    }
  };

  struct ExtrinsicParam {
    double d_1 = 0.0;
    double a_1 = 0.0;
    double phi_1 = 0.0;    // -pi~pi
    double theta_2 = 0.0;  // -pi~pi
    double d_2 = 0.0;
    double a_2 = 0.0;
    double phi_2 = 0.0;  // -pi~pi
  };

  const double one_three = 1.0 / 3.0;

  RotationLidarCalibration(Config config)
      : config_(config) {
    config_.Print();

    voxel_grid_ptr_ = std::make_unique<ConcurrentVoxelGrid<PointType>>(
        config_.downsample_size);
  };

  bool Optimization(const std::vector<std::shared_ptr<Plane>>& planes_array,
                    ExtrinsicParam& est_ext);

  void ComputeHessianJocabianResidual(
      const std::vector<std::shared_ptr<Plane>>& planes_array,
      const ExtrinsicParam& est_ext,
      Eigen::MatrixXd& hess,
      Eigen::VectorXd& jaco_T,
      double& residual);

  void ComputeResidual(const std::vector<std::shared_ptr<Plane>>& planes_array,
                       const ExtrinsicParam& est_ext,
                       bool compute_all,
                       double& residual);

  void RosbagToGroupPointArray(
      const std::vector<std::string>& bags_path,
      const std::string cloud_topic,
      const std::string encoder_topic,
      std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array);

  bool BuildVoxelMap(
      const std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array,
      const ExtrinsicParam& ext_param,
      std::vector<std::shared_ptr<Plane>>& planes_array);

  bool OutlierRemove(const std::vector<std::shared_ptr<Plane>>& planes_array,
                     const ExtrinsicParam& ext_param);

  static double AngleInterpolate(const double angle_prev,
                                 const double t_prev,
                                 const double angle_next,
                                 const double t_next,
                                 const double t_curr);

  bool ProcessRosbags(const std::vector<std::string>& bag_path_array,
                      const std::string& cloud_topic,
                      const LidarType lidar_type,
                      const std::string& encoder_topic,
                      const std::string& save_path);

  void ProcessSimCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_ptr,
      CloudMsg& cloud_msg);

  void ProcessLivoxCloud(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg_ptr,
      CloudMsg& cloud_msg);

  bool ComputeAlignRotation(
      const std::vector<std::shared_ptr<Point>>& point_array,
      const ExtrinsicParam& ext_param,
      Eigen::Matrix3d& R_W_B);

  template <typename T>
  bool HasInf(const T& p) {
    return (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z));
  }

  template <typename T>
  bool HasNan(const T& p) {
    return (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z));
  }

  template <typename T>
  bool IsNear(const T& p1, const T& p2) {
    return ((abs(p1.x - p2.x) < 1e-7) || (abs(p1.y - p2.y) < 1e-7) ||
            (abs(p1.z - p2.z) < 1e-7));
  }

  std::vector<std::string> GetFilesWitExtension(
      const std::string& database_path,
      const std::string& extension,
      const bool need_sort = true);

  void LoadDatabase(
      const std::string& database_path,
      std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array);

  bool IsDirectory(const std::string& result_path);

  void SaveResult(
      const std::string& result_path,
      const std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array,
      const ExtrinsicParam& calib_est_param,
      const ExtrinsicParam& uncalib_est_param);

  void LoadExtrinsicParam(const std::string& path, ExtrinsicParam& ext_param);

  std::vector<std::shared_ptr<
      std::unordered_map<VOXEL_LOCATION, std::shared_ptr<OctreeNode>>>>
      voxel_map_array_;

  Config config_;

  std::unique_ptr<ConcurrentVoxelGrid<PointType>> voxel_grid_ptr_;

  std::vector<Eigen::Matrix3d> R_W_B_array_;

  bool first_iter_convergence_ = false;
};