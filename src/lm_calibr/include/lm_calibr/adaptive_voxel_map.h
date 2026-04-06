#pragma once
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#define HASH_P 116101
#define MAX_N 10000000000
#define SMALL_EPS 1e-10

using PointType = pcl::PointXYZINormal;

class VOXEL_LOCATION {
 public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx)
      , y(vy)
      , z(vz) {}

  bool operator==(const VOXEL_LOCATION& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <>
struct hash<VOXEL_LOCATION> {
  size_t operator()(const VOXEL_LOCATION& s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    long long index_x, index_y, index_z;
    double cub_len = 0.125;
    index_x = int(std::round(std::floor((s.x) / cub_len + SMALL_EPS)));
    index_y = int(std::round(std::floor((s.y) / cub_len + SMALL_EPS)));
    index_z = int(std::round(std::floor((s.z) / cub_len + SMALL_EPS)));
    return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) +
            index_x) %
           MAX_N;
  }
};
}  // namespace std

struct Point {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_B = Eigen::Vector3d::Zero();
  double angle = 0.0;
  double angle_vel = 0.0;
  size_t group_idx = 0;
  size_t plane_idx = 0;
  size_t head_idx = 0;
  size_t tail_idx = 0;
};

struct Plane {
  std::vector<std::shared_ptr<Point>> points_array;
  bool is_valid = true;
  double residual = 0.0;
  size_t plane_idx = 0;
};

enum OctreeNodeState { UNKNOWN = 0, MID_NODE, PLANE };

class OctreeNode {
 public:
  struct Config {
    std::vector<double> eigen_threshold = {0.2, 0.1, 0.1};
    size_t max_layer = 2;
  };

  OctreeNode(Config config, size_t layer)
      : config_(config)
      , layer_(layer) {
    CHECK_EQ(config_.eigen_threshold.size(), (config_.max_layer + 1));

    sub_nodes_.resize(8, nullptr);

    std::uniform_int_distribution<int> dis(0, 254);
    ref_color_ = dis(seed);
  }

  void ProcessNode();

  bool JudgePlane();

  void SubDivide();

  void ExtractPlane(std::vector<std::shared_ptr<Plane>>& planes_array,
                    size_t& plane_count);

  void Display(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
               size_t& plane_count);

  Config config_;
  size_t layer_;
  OctreeNodeState octree_state_ = OctreeNodeState::UNKNOWN;
  Eigen::Vector3f voxel_center_ = Eigen::Vector3f::Zero();
  float quater_length_ = 0.0;
  float ref_color_ = 0.0;

  std::vector<std::shared_ptr<OctreeNode>> sub_nodes_;
  std::vector<std::shared_ptr<Point>> points_array_;

  static std::mt19937 seed;
};