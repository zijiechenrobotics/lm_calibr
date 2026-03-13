#include "lm_calibr/adaptive_voxel_map.h"

std::mt19937 OctreeNode::seed(0);

void OctreeNode::ProcessNode() {
  if (octree_state_ == OctreeNodeState::UNKNOWN) {
    if (points_array_.size() < 15) {
      octree_state_ = OctreeNodeState::MID_NODE;
      points_array_.clear();
      return;
    }

    if (JudgePlane()) {
      octree_state_ = OctreeNodeState::PLANE;
      return;
    } else {
      if (layer_ == config_.max_layer) {
        octree_state_ = OctreeNodeState::MID_NODE;
        points_array_.clear();
        return;
      }

      SubDivide();
    }
  }

  for (const auto& sub_node : sub_nodes_) {
    if (sub_node != nullptr) {
      sub_node->ProcessNode();
    }
  }
}

void OctreeNode::SubDivide() {
  for (const auto& point_ptr : points_array_) {
    size_t xyz[3] = {0, 0, 0};
    for (uint i = 0; i < 3; ++i) {
      if (point_ptr->p_B[i] > voxel_center_[i]) {
        xyz[i] = 1;
      }
    }
    size_t sub_node_idx = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    if (sub_nodes_[sub_node_idx] == nullptr) {
      sub_nodes_[sub_node_idx] =
          std::make_shared<OctreeNode>(config_, layer_ + 1);
      sub_nodes_[sub_node_idx]->voxel_center_[0] =
          voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      sub_nodes_[sub_node_idx]->voxel_center_[1] =
          voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      sub_nodes_[sub_node_idx]->voxel_center_[2] =
          voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      sub_nodes_[sub_node_idx]->quater_length_ = quater_length_ / 2.0;
    }

    sub_nodes_[sub_node_idx]->points_array_.push_back(point_ptr);
  }

  points_array_.clear();
}

bool OctreeNode::JudgePlane() {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& point_ptr : points_array_) {
    center += point_ptr->p_B;
    cov += point_ptr->p_B * point_ptr->p_B.transpose();
  }
  double N = static_cast<double>(points_array_.size());
  center /= N;
  cov = cov / N - center * center.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);

  double eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2];

  // ref https://arxiv.org/pdf/2305.00287
  if (eigen_ratio > config_.eigen_threshold[layer_]) {
    return false;
  }

  Eigen::Vector3d direct = saes.eigenvectors().col(0);
  double eva0 = saes.eigenvalues()[0];
  double sqr_eva0 = sqrt(eva0);
  Eigen::Vector3d center_turb = center + 5 * sqr_eva0 * direct;
  std::vector<std::vector<std::shared_ptr<Point>>> sub_node_point_array(8);
  for (const auto& point_ptr : points_array_) {
    size_t xyz[3] = {0, 0, 0};
    for (size_t k = 0; k < 3; k++) {
      if (point_ptr->p_B[k] > center_turb[k]) {
        xyz[k] = 1;
      }
    }

    size_t sub_node_idx = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    sub_node_point_array[sub_node_idx].push_back(point_ptr);
  }

  double ratios[2] = {1.0 / (3.0 * 3.0), 2.0 * 2.0};
  size_t num_all = 0, num_qua = 0;
  for (size_t i = 0; i < 8; i++)
    if (sub_node_point_array[i].size() > 10) {
      Eigen::Matrix3d sub_cov = Eigen::Matrix3d::Zero();
      Eigen::Vector3d sub_center = Eigen::Vector3d::Zero();
      for (const auto& point_ptr : sub_node_point_array[i]) {
        sub_cov += point_ptr->p_B * point_ptr->p_B.transpose();
        sub_center += point_ptr->p_B;
      }
      double sub_N = static_cast<double>(sub_node_point_array[i].size());
      sub_center /= sub_N;
      sub_cov = sub_cov / sub_N - sub_center * sub_center.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sub_cov);
      double child_eva0 = (saes.eigenvalues()[0]);
      if (child_eva0 > ratios[0] * eva0 && child_eva0 < ratios[1] * eva0) {
        num_qua++;
      }
      num_all++;
    }

  double prop = 1.0 * num_qua / num_all;

  if (prop < 0.5) {
    return false;
  }

  return true;
}

void OctreeNode::ExtractPlane(std::vector<std::shared_ptr<Plane>>& planes_array,
                              size_t& plane_count) {
  if (octree_state_ == OctreeNodeState::PLANE) {
    size_t plane_idx = planes_array.size();

    std::shared_ptr<Plane> plane_ptr(new Plane());
    plane_ptr->plane_idx = plane_idx;
    for (const auto& p_ptr : points_array_) {
      p_ptr->plane_idx = plane_idx;
      plane_ptr->points_array.push_back(p_ptr);
    }

    planes_array.push_back(plane_ptr);
    plane_count++;
  } else {
    for (const auto& sub_node : sub_nodes_) {
      if (sub_node != nullptr) {
        sub_node->ExtractPlane(planes_array, plane_count);
      }
    }
  }
}

void OctreeNode::Display(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                         size_t& plane_count) {
  if (octree_state_ == OctreeNodeState::UNKNOWN) {
    for (const auto& sub_node : sub_nodes_) {
      if (sub_node != nullptr) {
        sub_node->Display(cloud_ptr, plane_count);
      }
    }
  } else if (octree_state_ == OctreeNodeState::PLANE) {
    plane_count++;
    for (const auto& point_ptr : points_array_) {
      pcl::PointXYZI pt;
      pt.intensity = ref_color_;
      pt.x = point_ptr->p_B.x();
      pt.y = point_ptr->p_B.y();
      pt.z = point_ptr->p_B.z();

      cloud_ptr->push_back(pt);
    }
  }
}
