#pragma once

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <random>

template <typename PointType>
class ConcurrentVoxelGrid {
 public:
  struct Voxel {
    Voxel() {};

    std::vector<PointType> point_array;
  };

  ConcurrentVoxelGrid(double resolution)
      : resolution_(resolution)
      , inv_resolution_(1.0 / resolution) {};

  void AllocatePointCloud(
      const typename pcl::PointCloud<PointType>::Ptr& input_cloud);

  void ResetVoxelGrid();

  size_t ComputeHashIndex(const Eigen::Vector3d& point);

  void Filter(const typename pcl::PointCloud<PointType>::Ptr& input_cloud,
              typename pcl::PointCloud<PointType>::Ptr& output_cloud);

  void ResetResolution(double resolution) {
    resolution_ = resolution;
    inv_resolution_ = 1.0 / resolution;
  }

  using MyHashMap = tbb::concurrent_hash_map<size_t, std::shared_ptr<Voxel>>;
  using MyVector = tbb::concurrent_vector<std::shared_ptr<Voxel>>;

  std::shared_ptr<MyHashMap> voxel_map_ptr_;
  std::shared_ptr<MyVector> voxel_array_ptr_;

  double resolution_ = 1.0;
  double inv_resolution_ = 1.0 / resolution_;

  size_t HASH_P_{116101};
  size_t MAX_N_{10000000000};
};

template <typename PointType>
size_t ConcurrentVoxelGrid<PointType>::ComputeHashIndex(
    const Eigen::Vector3d& point) {
  Eigen::Vector3i grid_idx =
      (point * inv_resolution_).array().floor().cast<int>();
  return size_t(((grid_idx.x()) * 73856093) ^ ((grid_idx.y()) * 471943) ^
                ((grid_idx.z()) * 83492791)) %
         10000000;
}

template <typename PointType>
void ConcurrentVoxelGrid<PointType>::AllocatePointCloud(
    const typename pcl::PointCloud<PointType>::Ptr& input_cloud) {
  voxel_map_ptr_ = std::make_shared<MyHashMap>();
  voxel_array_ptr_ = std::make_shared<MyVector>();
  voxel_array_ptr_->reserve(input_cloud->size());

  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, input_cloud->size()),
      [&, this](tbb::blocked_range<size_t> r) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          Eigen::Vector3d point =
              input_cloud->points[i].getVector3fMap().template cast<double>();
          size_t hash_idx = ComputeHashIndex(point);

          typename MyHashMap::accessor acceessor;
          voxel_map_ptr_->insert(acceessor, hash_idx);

          if (acceessor->second == nullptr) {
            acceessor->second = std::make_shared<Voxel>();

            acceessor->second->point_array.push_back(input_cloud->points[i]);

            voxel_array_ptr_->emplace_back(acceessor->second);
          } else {
            acceessor->second->point_array.push_back(input_cloud->points[i]);
          }
        }
      });
}

template <typename PointType>
void ConcurrentVoxelGrid<PointType>::Filter(
    const typename pcl::PointCloud<PointType>::Ptr& input_cloud,
    typename pcl::PointCloud<PointType>::Ptr& output_cloud) {
  AllocatePointCloud(input_cloud);

  output_cloud->clear();
  for (const auto& voxel_ptr : *voxel_array_ptr_) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, voxel_ptr->point_array.size() - 1);
    PointType p = voxel_ptr->point_array[dis(gen)];
    output_cloud->push_back(p);
  }

  ResetVoxelGrid();
}

template <typename PointType>
void ConcurrentVoxelGrid<PointType>::ResetVoxelGrid() {
  voxel_map_ptr_->clear();
  voxel_array_ptr_->clear();
}
