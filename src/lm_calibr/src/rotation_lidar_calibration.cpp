#include "lm_calibr/rotation_lidar_calibration.h"

void RotationLidarCalibration::ComputeHessianJocabianResidual(
    const std::vector<std::shared_ptr<Plane>>& planes_array,
    const ExtrinsicParam& est_ext,
    Eigen::MatrixXd& hess,
    Eigen::VectorXd& jaco_T,
    double& residual) {
  hess.setZero();
  jaco_T.setZero();
  residual = 0.0;

  Eigen::Matrix3d R_x_phi_1 =
      Eigen::AngleAxisd(est_ext.phi_1, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d R_z_theta_2 =
      Eigen::AngleAxisd(est_ext.theta_2, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d R_x_phi_2 =
      Eigen::AngleAxisd(est_ext.phi_2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Vector3d t_2(est_ext.a_1, 0.0, est_ext.d_1);
  Eigen::Vector3d t_1(est_ext.a_2, 0.0, est_ext.d_2);

  for (size_t i = 0; i < planes_array.size(); ++i) {
    if (!planes_array[i]->is_valid) {
      continue;
    }

    const std::vector<std::shared_ptr<Point>>& points =
        planes_array[i]->points_array;

    std::vector<Eigen::Vector3d> pij_array(points.size());
    std::vector<Eigen::MatrixXd> d_pij_d_x_array(points.size());
    Eigen::Vector3d q_i = Eigen::Vector3d::Zero();
    Eigen::Matrix3d A_i = Eigen::Matrix3d::Zero();

    // tbb version
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_q_A = {
        Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    auto reduced_q_A = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, points.size()),
        init_q_A,
        [&, this](const tbb::blocked_range<size_t>& r,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> local_result)
            -> std::pair<Eigen::Vector3d, Eigen::Matrix3d> {
          for (size_t j = r.begin(); j < r.end(); ++j) {
            Eigen::Matrix3d R_z_theta_1 =
                Eigen::AngleAxisd(points[j]->angle, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();

            Eigen::Matrix3d Rw_Rz_theta1 =
                R_W_B_array_[points[j]->group_idx] * R_z_theta_1;
            Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1 = Rw_Rz_theta1 * R_x_phi_1;
            Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2 =
                Rw_Rz_theta1_Rx_phi1 * R_z_theta_2;
            Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 =
                Rw_Rz_theta1_Rx_phi1_Rz_theta2 * R_x_phi_2;

            Eigen::Matrix3d d_pij_d_theta = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d d_pij_d_phi = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d d_pij_d_d = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d d_pij_d_a = Eigen::Matrix3d::Zero();

            if (config_.DH_type == 0) {
              d_pij_d_theta = -Rw_Rz_theta1_Rx_phi1_Rz_theta2 *
                              Sophus::SO3d::hat(R_x_phi_2 * points[j]->p + t_1);
              d_pij_d_phi = -Rw_Rz_theta1_Rx_phi1 *
                            Sophus::SO3d::hat(R_z_theta_2 *
                                              (R_x_phi_2 * points[j]->p + t_1));
              d_pij_d_d = Rw_Rz_theta1_Rx_phi1_Rz_theta2;
              d_pij_d_a = Rw_Rz_theta1;
            } else if (config_.DH_type == 1) {
              d_pij_d_theta = -Rw_Rz_theta1_Rx_phi1_Rz_theta2 *
                              Sophus::SO3d::hat(R_x_phi_2 * points[j]->p + t_1);
              d_pij_d_phi = -Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 *
                            Sophus::SO3d::hat(points[j]->p);
              d_pij_d_d = Rw_Rz_theta1_Rx_phi1_Rz_theta2;
              d_pij_d_a = Rw_Rz_theta1_Rx_phi1_Rz_theta2;
            } else {
              LOG(WARNING) << "error DH type" << std::endl;
            }

            Eigen::MatrixXd d_pij_d_x(3, 4);
            d_pij_d_x.block<3, 1>(0, 0) = d_pij_d_phi.col(0);
            d_pij_d_x.block<3, 1>(0, 1) = d_pij_d_theta.col(2);
            d_pij_d_x.block<3, 1>(0, 2) = d_pij_d_a.col(0);
            d_pij_d_x.block<3, 1>(0, 3) = d_pij_d_d.col(2);
            d_pij_d_x_array[j] = d_pij_d_x;

            Eigen::Vector3d t_W_L =
                Rw_Rz_theta1_Rx_phi1_Rz_theta2 * t_1 + Rw_Rz_theta1 * t_2;
            pij_array[j] =
                Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 * points[j]->p + t_W_L;

            local_result.first += pij_array[j];
            local_result.second += pij_array[j] * pij_array[j].transpose();
          }
          return local_result;
        },
        [](const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& x,
           const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& y)
            -> std::pair<Eigen::Vector3d, Eigen::Matrix3d> {
          return {x.first + y.first, x.second + y.second};
        });

    q_i = reduced_q_A.first;
    A_i = reduced_q_A.second;

    double N_i = static_cast<double>(points.size());
    q_i /= N_i;
    A_i = A_i / N_i - q_i * q_i.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(A_i);
    Eigen::Vector3d eigen_value = saes.eigenvalues();
    Eigen::Matrix3d U = saes.eigenvectors();
    Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};

    residual += eigen_value[0];

    Eigen::Matrix3d u3_u3_T(u[0] * u[0].transpose());

    std::vector<Eigen::Matrix3d> F_pij_m_3_partial(3);
    for (size_t m = 0; m < 3; ++m) {
      if (m == 0) {
        F_pij_m_3_partial[m].setZero();
      } else {
        Eigen::Matrix3d u3_um_T = u[0] * u[m].transpose();
        F_pij_m_3_partial[m] = 1.0 / N_i / (eigen_value[0] - eigen_value[m]) *
                               (u3_um_T + u3_um_T.transpose());
      }
    }

    Eigen::Matrix3d u3_u3_T_same = (N_i - 1) / N_i * u3_u3_T;
    Eigen::Matrix3d u3_u3_T_diff = -1.0 / N_i * u3_u3_T;

    // tbb version
    std::pair<Eigen::Vector4d, Eigen::Matrix4d> init_jaco_hess = {
        Eigen::Vector4d::Zero(), Eigen::Matrix4d::Zero()};
    auto reduced_jaco_hess = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, points.size()),
        init_jaco_hess,
        [&, this](const tbb::blocked_range<size_t>& r,
                  std::pair<Eigen::Vector4d, Eigen::Matrix4d> local_result)
            -> std::pair<Eigen::Vector4d, Eigen::Matrix4d> {
          Eigen::Vector4d temp_jaco_T = Eigen::Vector4d::Zero();
          Eigen::Matrix4d temp_hess = Eigen::Matrix4d::Zero();

          for (size_t j = r.begin(); j < r.end(); ++j) {
            // compute jacobian
            Eigen::Vector3d pij_qi = pij_array[j] - q_i;
            Eigen::Vector3d d_lambda_d_pij_T = 2.0 / N_i * u3_u3_T * pij_qi;

            temp_jaco_T += d_pij_d_x_array[j].transpose() * d_lambda_d_pij_T;

            // compute hessian
            Eigen::Matrix3d F_pij_3 = Eigen::Matrix3d::Zero();
            for (size_t m = 0; m < 3; ++m) {
              F_pij_3.block<1, 3>(m, 0) =
                  pij_qi.transpose() * F_pij_m_3_partial[m];
            }

            Eigen::Matrix3d U_F_pij_3 = U * F_pij_3;

            for (size_t k = j; k < points.size(); ++k) {
              Eigen::Vector3d pik_qi = pij_array[k] - q_i;
              Eigen::Matrix3d temp_H = u[0] * pik_qi.transpose() * U_F_pij_3 +
                                       u[0].dot(pik_qi) * U_F_pij_3;

              if (k == j) {
                temp_H += u3_u3_T_same;
              } else {
                temp_H += u3_u3_T_diff;
              }
              temp_H = 2.0 / N_i * temp_H;

              if (k == j) {
                temp_hess += d_pij_d_x_array[k].transpose() * temp_H *
                             d_pij_d_x_array[k];
              } else {
                Eigen::Matrix4d H = d_pij_d_x_array[j].transpose() * temp_H *
                                    d_pij_d_x_array[k];
                temp_hess += H + H.transpose();
              }
            }
          }

          local_result.first += temp_jaco_T;
          local_result.second += temp_hess;

          return local_result;
        },
        [](const std::pair<Eigen::Vector4d, Eigen::Matrix4d>& left,
           const std::pair<Eigen::Vector4d, Eigen::Matrix4d>& right)
            -> std::pair<Eigen::Vector4d, Eigen::Matrix4d> {
          return {left.first + right.first, left.second + right.second};
        });

    jaco_T += reduced_jaco_hess.first;
    hess += reduced_jaco_hess.second;
  }
}

void RotationLidarCalibration::ComputeResidual(
    const std::vector<std::shared_ptr<Plane>>& planes_array,
    const ExtrinsicParam& est_ext,
    bool compute_all,
    double& residual) {
  residual = 0.0;

  Eigen::Matrix3d R_x_phi_1 =
      Eigen::AngleAxisd(est_ext.phi_1, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d R_z_theta_2 =
      Eigen::AngleAxisd(est_ext.theta_2, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d R_x_phi_2 =
      Eigen::AngleAxisd(est_ext.phi_2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Vector3d t_2(est_ext.a_1, 0.0, est_ext.d_1);
  Eigen::Vector3d t_1(est_ext.a_2, 0.0, est_ext.d_2);

  Eigen::Vector3d p_ij;

  for (size_t i = 0; i < planes_array.size(); ++i) {
    // skip outlier plane features
    if (!compute_all && !planes_array[i]->is_valid) {
      continue;
    }

    const std::vector<std::shared_ptr<Point>>& points =
        planes_array[i]->points_array;

    Eigen::Vector3d q_i = Eigen::Vector3d::Zero();
    Eigen::Matrix3d A_i = Eigen::Matrix3d::Zero();

    // tbb version
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_q_A = {
        Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    auto reduced_q_A = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, points.size()),
        init_q_A,
        [&](const tbb::blocked_range<size_t>& r,
            std::pair<Eigen::Vector3d, Eigen::Matrix3d> local_result)
            -> std::pair<Eigen::Vector3d, Eigen::Matrix3d> {
          for (size_t j = r.begin(); j < r.end(); ++j) {
            Eigen::Matrix3d R_z_theta_1 =
                Eigen::AngleAxisd(points[j]->angle, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();

            Eigen::Matrix3d Rw_Rz_theta1 =
                R_W_B_array_[points[j]->group_idx] * R_z_theta_1;
            Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2 =
                Rw_Rz_theta1 * R_x_phi_1 * R_z_theta_2;
            Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 =
                Rw_Rz_theta1_Rx_phi1_Rz_theta2 * R_x_phi_2;
            Eigen::Vector3d p_ij =
                Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 * points[j]->p +
                Rw_Rz_theta1_Rx_phi1_Rz_theta2 * t_1 + Rw_Rz_theta1 * t_2;

            local_result.first += p_ij;
            local_result.second += p_ij * p_ij.transpose();
          }
          return local_result;
        },
        [](const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& x,
           const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& y)
            -> std::pair<Eigen::Vector3d, Eigen::Matrix3d> {
          return {x.first + y.first, x.second + y.second};
        });

    q_i = reduced_q_A.first;
    A_i = reduced_q_A.second;

    double N_i = static_cast<double>(points.size());
    q_i /= N_i;
    A_i = A_i / N_i - q_i * q_i.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(A_i);
    Eigen::Vector3d eigen_value = saes.eigenvalues();
    residual += eigen_value[0];
    planes_array[i]->residual = eigen_value[0];
  }
}

bool RotationLidarCalibration::Optimization(
    const std::vector<std::shared_ptr<Plane>>& planes_array,
    ExtrinsicParam& est_ext) {
  double u = 0.1;
  double v = 2;
  Eigen::MatrixXd D(4, 4);
  D.setIdentity();
  Eigen::MatrixXd hess(4, 4);
  Eigen::VectorXd jaco_T(4);
  Eigen::VectorXd dx(4);
  double residual_1 = 0.0;
  double residual_2 = 0.0;
  double q = 0.0;
  bool need_calc_hess = true;

  ExtrinsicParam temp_ext = est_ext;
  first_iter_convergence_ = false;
  for (size_t i = 0; i < config_.max_iter; ++i) {
    if (need_calc_hess) {
      // compute hessian jacobian
      ComputeHessianJocabianResidual(
          planes_array, est_ext, hess, jaco_T, residual_1);
    }

    // std::cout << "iter " << i << std::endl;
    // std::cout << "hess: " << std::endl << hess << std::endl;
    // std::cout << "jaco: " << jaco_T.transpose() << std::endl;

    // print observability
    Eigen::EigenSolver<Eigen::Matrix2d> rot_saes(hess.block<2, 2>(0, 0));
    Eigen::Vector2d rot_eigen = rot_saes.eigenvalues().real();
    LOG(INFO) << "phi: " << rot_eigen[0] << ", theta: " << rot_eigen[1];
    Eigen::EigenSolver<Eigen::Matrix2d> trans_saes(hess.block<2, 2>(2, 2));
    Eigen::Vector2d trans_eigen = trans_saes.eigenvalues().real();
    LOG(INFO) << "a: " << trans_eigen[0] << ", d: " << trans_eigen[1];
    LOG(INFO) << "rot_min: " << rot_eigen.minCoeff() << std::endl
              << "trans_min: " << trans_eigen.minCoeff();

    D.diagonal() = hess.diagonal();
    dx = (hess + u * D).ldlt().solve(-jaco_T);
    LOG(INFO) << "dx: " << dx.transpose();
    if (i == 0 && dx.cwiseAbs().maxCoeff() < 1e-4) {
      first_iter_convergence_ = true;
      LOG(INFO) << "first iter convergence";
    }

    // update base on DH_type
    if (config_.DH_type == 0) {
      temp_ext.phi_1 = est_ext.phi_1 + dx(0, 0);
      if (temp_ext.phi_1 < -M_PI) {
        temp_ext.phi_1 += M_PI * 2.0;
      } else if (temp_ext.phi_1 > M_PI) {
        temp_ext.phi_1 -= M_PI * 2.0;
      }

      temp_ext.theta_2 = est_ext.theta_2 + dx(1, 0);
      if (temp_ext.theta_2 < -M_PI) {
        temp_ext.theta_2 += M_PI * 2.0;
      } else if (temp_ext.theta_2 > M_PI) {
        temp_ext.theta_2 -= M_PI * 2.0;
      }

      temp_ext.a_1 = est_ext.a_1 + dx(2, 0);

      temp_ext.d_2 = est_ext.d_2 + dx(3, 0);
    } else if (config_.DH_type == 1) {
      temp_ext.phi_2 = est_ext.phi_2 + dx(0, 0);
      if (temp_ext.phi_2 < -M_PI) {
        temp_ext.phi_2 += M_PI * 2.0;
      } else if (temp_ext.phi_2 > M_PI) {
        temp_ext.phi_2 -= M_PI * 2.0;
      }

      temp_ext.theta_2 = est_ext.theta_2 + dx(1, 0);
      if (temp_ext.theta_2 < -M_PI) {
        temp_ext.theta_2 += M_PI * 2.0;
      } else if (temp_ext.theta_2 > M_PI) {
        temp_ext.theta_2 -= M_PI * 2.0;
      }

      temp_ext.a_2 = est_ext.a_2 + dx(2, 0);

      temp_ext.d_2 = est_ext.d_2 + dx(3, 0);
    } else {
      LOG(ERROR) << "error DH type" << std::endl;
    }

    double q1 = 0.5 * dx.dot(u * D * dx - jaco_T);
    // compute residual
    ComputeResidual(planes_array, temp_ext, false, residual_2);
    q = (residual_1 - residual_2);

    LOG(INFO) << "iter " << i << " ( " << residual_1 << " " << residual_2
              << " )"
              << " u: " << u << " v: " << v << " q: " << q << std::endl;

    if (q > 0) {
      est_ext = temp_ext;

      q = q / q1;
      v = 2;
      q = 1 - pow(2 * q - 1, 3);
      u *= (q < one_three ? one_three : q);
      need_calc_hess = true;
    } else {
      u = u * v;
      v = 2 * v;
      need_calc_hess = false;
    }

    if (fabs((residual_1 - residual_2)) / residual_1 < 1e-6) {
      break;
    }
  }

  return true;
}

bool RotationLidarCalibration::BuildVoxelMap(
    const std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array,
    const ExtrinsicParam& ext_param,
    std::vector<std::shared_ptr<Plane>>& planes_array) {
  Eigen::Matrix3d R_x_phi_1 =
      Eigen::AngleAxisd(ext_param.phi_1, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d R_z_theta_2 =
      Eigen::AngleAxisd(ext_param.theta_2, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d R_x_phi_2 =
      Eigen::AngleAxisd(ext_param.phi_2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Vector3d t_2(ext_param.a_1, 0.0, ext_param.d_1);
  Eigen::Vector3d t_1(ext_param.a_2, 0.0, ext_param.d_2);

  voxel_map_array_.resize(group_point_array.size());
  R_W_B_array_.resize(group_point_array.size());

  for (size_t i = 0; i < group_point_array.size(); ++i) {
    R_W_B_array_[i] = Eigen::Matrix3d::Identity();

    const auto& point_array = group_point_array[i];
    voxel_map_array_[i] = std::make_shared<
        std::unordered_map<VOXEL_LOCATION, std::shared_ptr<OctreeNode>>>();
    const auto& voxel_map_ptr = voxel_map_array_[i];

    // allocate the laser points to each voxel
    for (const auto& point_ptr : point_array) {
      Eigen::Matrix3d R_z_theta_1 =
          Eigen::AngleAxisd(point_ptr->angle, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();

      Eigen::Matrix3d Rw_Rz_theta1 =
          R_W_B_array_[point_ptr->group_idx] * R_z_theta_1;
      Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2 =
          Rw_Rz_theta1 * R_x_phi_1 * R_z_theta_2;
      Eigen::Matrix3d Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 =
          Rw_Rz_theta1_Rx_phi1_Rz_theta2 * R_x_phi_2;
      point_ptr->p_B = Rw_Rz_theta1_Rx_phi1_Rz_theta2_Rx_phi2 * point_ptr->p +
                       Rw_Rz_theta1_Rx_phi1_Rz_theta2 * t_1 +
                       Rw_Rz_theta1 * t_2;

      float loc_xyz[3];
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = point_ptr->p_B[j] / config_.max_voxel_size;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
      VOXEL_LOCATION position(
          (int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

      auto iter = voxel_map_ptr->find(position);
      if (iter != voxel_map_ptr->end()) {
        iter->second->points_array_.push_back(point_ptr);
      } else {
        OctreeNode::Config octree_node_config;
        octree_node_config.max_layer = config_.max_layer;
        octree_node_config.eigen_threshold = config_.eigen_threshold;
        std::shared_ptr<OctreeNode> octree_node_ptr(
            new OctreeNode(octree_node_config, 0));
        octree_node_ptr->points_array_.push_back(point_ptr);
        octree_node_ptr->voxel_center_.x() =
            (0.5 + position.x) * config_.max_voxel_size;
        octree_node_ptr->voxel_center_.y() =
            (0.5 + position.y) * config_.max_voxel_size;
        octree_node_ptr->voxel_center_.z() =
            (0.5 + position.z) * config_.max_voxel_size;
        octree_node_ptr->quater_length_ = config_.max_voxel_size / 4.0;
        voxel_map_ptr->insert(std::make_pair(position, octree_node_ptr));
      }
    }

    for (const auto& voxel_ptr : *voxel_map_ptr) {
      voxel_ptr.second->ProcessNode();
    }

    size_t plane_count = 0;
    for (const auto& voxel_ptr : *voxel_map_ptr) {
      voxel_ptr.second->ExtractPlane(planes_array, plane_count);
    }

    LOG(INFO) << "[RotationLidarCalibration::BuildVoxelMap] group: " << i
              << ", plane_count: " << plane_count << std::endl;
  }

  return true;
}

bool RotationLidarCalibration::OutlierRemove(
    const std::vector<std::shared_ptr<Plane>>& planes_array,
    const ExtrinsicParam& ext_param) {
  double total_residual = 0.0;
  ComputeResidual(planes_array, ext_param, true, total_residual);

  std::vector<double> residuals;
  residuals.resize(planes_array.size());
  for (size_t i = 0; i < planes_array.size(); ++i) {
    residuals[i] = planes_array[i]->residual;
  }

  std::sort(residuals.begin(), residuals.end());
  double threshold = residuals[std::floor((1 - config_.outlier_remove_ratio) *
                                          planes_array.size()) -
                               1];

  size_t remain_size = 0;
  for (const auto& plane_ptr : planes_array) {
    if (plane_ptr->residual > threshold) {
      plane_ptr->is_valid = false;
    } else {
      remain_size++;
    }
  }

  LOG(INFO) << "[OutlierRemove] reject threshold: " << threshold
            << ", origin_size: " << planes_array.size()
            << ", remain_size: " << remain_size;

  return true;
}

/**
 * @brief Linear interpolation for periodic variables ranging from 0 to 2pi
 *
 * @param angle_prev
 * @param t_prev
 * @param angle_next
 * @param t_next
 * @param t_curr
 * @return double
 */
double RotationLidarCalibration::AngleInterpolate(const double angle_prev,
                                                  const double t_prev,
                                                  const double angle_next,
                                                  const double t_next,
                                                  const double t_curr) {
  double delta = angle_next - angle_prev;
  if (delta > M_PI) {
    delta -= TWO_PI;
  } else if (delta < -M_PI) {
    delta += TWO_PI;
  }

  double unwrapped_angle = angle_prev + delta;

  double interp_angle =
      angle_prev +
      (unwrapped_angle - angle_prev) * ((t_curr - t_prev) / (t_next - t_prev));

  double w = std::fmod(interp_angle, TWO_PI);
  if (w < 0) {
    w += TWO_PI;
  }
  return w;
}

bool RotationLidarCalibration::ProcessRosbags(
    const std::vector<std::string>& bag_path_array,
    const std::string& cloud_topic,
    const LidarType lidar_type,
    const std::string& encoder_topic,
    const std::string& save_path) {
  LOG(INFO) << "ProcessRosbags, bag.size(): " << bag_path_array.size();
  for (size_t i = 0; i < bag_path_array.size(); ++i) {
    LOG(INFO) << "open ros2 bag path: " << bag_path_array[i];
    rosbag2_cpp::Reader reader;
    reader.open(bag_path_array[i]);

    std::deque<EncoderMsg> encoder_msg_array;
    std::deque<CloudMsg> cloud_msg_array;

    rclcpp::Serialization<sensor_msgs::msg::JointState> joint_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serializer;

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      const std::string topic_name = bag_msg->topic_name;

      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);

      if (topic_name == encoder_topic) {
        sensor_msgs::msg::JointState msg;
        joint_serializer.deserialize_message(&serialized_msg, &msg);

        EncoderMsg encoder_msg;
        encoder_msg.angle = msg.position[0];
        encoder_msg.angle_vel = msg.velocity[0];
        encoder_msg.timestamp = rclcpp::Time(msg.header.stamp).seconds();

        encoder_msg_array.push_back(encoder_msg);
      }

      else if (topic_name == cloud_topic) {
        CloudMsg cloud_msg;

        if (lidar_type == LidarType::SIM) {
          sensor_msgs::msg::PointCloud2 msg;
          cloud_serializer.deserialize_message(&serialized_msg, &msg);

          auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
          ProcessSimCloud(msg_ptr, cloud_msg);
        } else if (lidar_type == LidarType::LIVOX) {
          livox_ros_driver2::msg::CustomMsg msg;
          livox_serializer.deserialize_message(&serialized_msg, &msg);

          auto msg_ptr =
              std::make_shared<livox_ros_driver2::msg::CustomMsg>(msg);
          ProcessLivoxCloud(msg_ptr, cloud_msg);
        } else {
          LOG(ERROR) << "error lidar type";
          return false;
        }

        cloud_msg_array.push_back(cloud_msg);
      }
    }

    if (encoder_msg_array.empty()) {
      LOG(ERROR) << "encoder_msg_array is empty" << std::endl;
      return false;
    }

    if (cloud_msg_array.empty()) {
      LOG(ERROR) << "cloud_msg_array is empty" << std::endl;
      return false;
    }

    std::sort(encoder_msg_array.begin(),
              encoder_msg_array.end(),
              [](const EncoderMsg& a, const EncoderMsg& b) {
                return a.timestamp < b.timestamp;
              });

    double begin_time = cloud_msg_array.front().timestamp;
    begin_time += config_.rosbag_skip;
    LOG(INFO) << "rosbag_skip: " << config_.rosbag_skip;

    double acc_angle = 0.0;
    bool is_first_flag = true;
    double prev_point_anlge = 0.0;
    bool process_finshed = false;
    double finsh_threshold;
    if (config_.DH_type == 0) {
      finsh_threshold = config_.angle_threshold;
    } else if (config_.DH_type == 1) {
      finsh_threshold = config_.angle_threshold;
    } else {
      LOG(ERROR) << "error DH_type: " << config_.DH_type;
      return false;
    }

    // interpolate the encoder data for each laser point
    pcl::PointCloud<PointType>::Ptr point_encoder_cloud(
        new pcl::PointCloud<PointType>());
    PointType point_encoder;
    for (const auto& cloud_msg : cloud_msg_array) {
      double bag_time = cloud_msg.timestamp;

      if (bag_time < begin_time) {
        continue;
      }

      bool is_first_point = true;

      for (const auto& pt : cloud_msg.cloud_ptr->points) {
        // pt.curvature: the time that relative to rosbag time, unit: s
        double pt_time = bag_time + pt.curvature;

        if (pt_time < encoder_msg_array.front().timestamp) {
          continue;
        }
        if (pt_time > encoder_msg_array.back().timestamp) {
          continue;
        }

        auto it_upper =
            std::lower_bound(encoder_msg_array.begin(),
                             encoder_msg_array.end(),
                             pt_time,
                             [](const EncoderMsg& msg, double time) {
                               return msg.timestamp < time;
                             });

        if (it_upper == encoder_msg_array.begin()) {
          // LOG(INFO) << "pt find upper_encoder is encoder_msg_array.begin(), "
          //              "skip this pt";
          continue;
        }

        auto it_lower = it_upper - 1;

        double pt_angle = AngleInterpolate(it_lower->angle,
                                           it_lower->timestamp,
                                           it_upper->angle,
                                           it_upper->timestamp,
                                           pt_time);
        double pt_angle_vel = it_lower->angle_vel +
                              (it_upper->angle_vel - it_lower->angle_vel) *
                                  ((pt_time - it_lower->timestamp) /
                                   (it_upper->timestamp - it_lower->timestamp));

        if (std::abs(pt_angle_vel) < 0.1) {
          continue;
        }

        point_encoder.x = pt.x;
        point_encoder.y = pt.y;
        point_encoder.z = pt.z;
        point_encoder.normal_x = pt_angle;
        point_encoder.normal_y = pt_angle_vel;
        point_encoder.intensity = pt.intensity;
        point_encoder.curvature = pt_time;

        point_encoder_cloud->push_back(point_encoder);

        if (is_first_point) {
          if (is_first_flag) {
            is_first_flag = false;
          } else {
            double diff = pt_angle - prev_point_anlge;
            if (diff > M_PI) {
              diff -= TWO_PI;
            } else if (diff < -M_PI) {
              diff += TWO_PI;
            }

            acc_angle += diff;

            // the process ends when the angle accumulation exceeds the
            // threshold to avoid too many point
            if (std::abs(acc_angle) >= finsh_threshold) {
              process_finshed = true;
              LOG(INFO) << "finsh angle: " << pt_angle << std::endl;
            }
          }

          prev_point_anlge = pt_angle;
          is_first_point = false;
        }
      }

      if (process_finshed) {
        LOG(INFO) << "process finsh, acc_angle: " << acc_angle
                  << ", time_diff: " << bag_time - begin_time;
        break;
      }
    }

    std::string file_name = save_path + "/" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFileASCII(file_name, *point_encoder_cloud);
  }

  return true;
}

void RotationLidarCalibration::ProcessSimCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg_ptr,
    CloudMsg& cloud_msg) {
  cloud_msg.timestamp = cloud_msg_ptr->header.stamp.sec +
                        cloud_msg_ptr->header.stamp.nanosec * 1e-9;
  cloud_msg.cloud_ptr.reset(new pcl::PointCloud<PointType>());

  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*cloud_msg_ptr, *cloud_ptr);

  voxel_grid_ptr_->Filter(cloud_ptr, cloud_msg.cloud_ptr);
}

void RotationLidarCalibration::ProcessLivoxCloud(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg_ptr,
    CloudMsg& cloud_msg) {
  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
  for (size_t i = 1; i < msg_ptr->point_num; ++i) {
    if (((msg_ptr->points[i].tag & 0x30) == 0x10 ||
         (msg_ptr->points[i].tag & 0x30) == 0x00) &&
        !HasInf(msg_ptr->points[i]) && !HasNan(msg_ptr->points[i]) &&
        !IsNear(msg_ptr->points[i], msg_ptr->points[i - 1])) {
      // distance filter
      double range = std::sqrt(msg_ptr->points[i].x * msg_ptr->points[i].x +
                               msg_ptr->points[i].y * msg_ptr->points[i].y +
                               msg_ptr->points[i].z * msg_ptr->points[i].z);
      if (range < config_.min_range || range > config_.max_range) {
        continue;
      }

      PointType pt;
      pt.normal_x = 0;
      pt.normal_y = 0;
      pt.normal_z = 0;
      pt.x = msg_ptr->points[i].x;
      pt.y = msg_ptr->points[i].y;
      pt.z = msg_ptr->points[i].z;
      pt.intensity = msg_ptr->points[i].reflectivity;
      pt.curvature = msg_ptr->points[i].offset_time * 1e-9;
      cloud_ptr->push_back(pt);
    }
  }

  cloud_msg.timestamp =
      msg_ptr->header.stamp.sec + msg_ptr->header.stamp.nanosec * 1e-9;
  cloud_msg.cloud_ptr.reset(new pcl::PointCloud<PointType>());
  voxel_grid_ptr_->Filter(cloud_ptr, cloud_msg.cloud_ptr);
}

std::vector<std::string> RotationLidarCalibration::GetFilesWitExtension(
    const std::string& database_path,
    const std::string& extension,
    const bool need_sort) {
  std::vector<std::string> files;

  try {
    for (const auto& entry :
         std::filesystem::directory_iterator(database_path)) {
      if (entry.path().extension() == extension) {
        files.push_back(entry.path().string());
      }
    }

    if (need_sort) {
      std::sort(files.begin(), files.end());
    }
  } catch (const std::filesystem::filesystem_error& e) {
    LOG(ERROR) << "Filesystem error: " << e.what() << std::endl;
  }

  return files;
}

void RotationLidarCalibration::LoadDatabase(
    const std::string& database_path,
    std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array) {
  std::vector<std::string> group_path_array =
      GetFilesWitExtension(database_path, ".pcd", true);

  LOG(INFO) << "[RotationLidarCalibration::LoadDatabase] pcds: " << std::endl;
  for (const auto& path : group_path_array) {
    LOG(INFO) << "\t" << path << std::endl;
  }

  size_t total_point = 0;
  group_point_array.resize(group_path_array.size());
  for (size_t i = 0; i < group_path_array.size(); ++i) {
    pcl::PointCloud<PointType>::Ptr point_encoder_cloud(
        new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile(group_path_array[i], *point_encoder_cloud) == -1) {
      LOG(ERROR) << "failed to open: " << group_path_array[i] << std::endl;
    }

    for (const auto& p : point_encoder_cloud->points) {
      std::shared_ptr<Point> point_ptr(new Point());
      point_ptr->p = Eigen::Vector3d(p.x, p.y, p.z);
      point_ptr->angle = p.normal_x;
      point_ptr->angle_vel = p.normal_y;
      point_ptr->group_idx = i;

      group_point_array[i].push_back(point_ptr);

      // count = 0;
      total_point++;
    }
  }
  LOG(INFO) << "[RotationLidarCalibration::LoadDatabase] total_point: "
            << total_point << std::endl;
}

bool RotationLidarCalibration::IsDirectory(const std::string& result_path) {
  bool have_result = false;
  if (std::filesystem::exists(result_path) &&
      std::filesystem::is_directory(result_path)) {
    have_result = true;
  }

  return have_result;
}

void RotationLidarCalibration::SaveResult(
    const std::string& result_path,
    const std::vector<std::vector<std::shared_ptr<Point>>>& group_point_array,
    const ExtrinsicParam& calib_est_param,
    const ExtrinsicParam& uncalib_est_param) {
  bool have_result = IsDirectory(result_path);

  size_t folder_count = 0;
  if (have_result) {
    for (const auto& entry : std::filesystem::directory_iterator(result_path)) {
      if (entry.is_directory()) {
        folder_count++;
      }
    }
    LOG(INFO) << "[SaveResult] It has been calibrated " << folder_count
              << " times." << std::endl;
  } else {
    LOG(INFO) << "[SaveResult] It has not been calibrated" << std::endl;
  }

  std::string curr_result_path =
      result_path + "/iter_" + std::to_string(folder_count + 1);
  std::filesystem::create_directories(curr_result_path);

  YAML::Emitter result_emitter;
  result_emitter << YAML::BeginMap;
  result_emitter << YAML::Key << "d_1" << YAML::Value << calib_est_param.d_1;
  result_emitter << YAML::Key << "a_1" << YAML::Value << calib_est_param.a_1;
  result_emitter << YAML::Key << "phi_1" << YAML::Value
                 << calib_est_param.phi_1;
  result_emitter << YAML::Key << "theta_2" << YAML::Value
                 << calib_est_param.theta_2;
  result_emitter << YAML::Key << "d_2" << YAML::Value << calib_est_param.d_2;
  result_emitter << YAML::Key << "a_2" << YAML::Value << calib_est_param.a_2;
  result_emitter << YAML::Key << "phi_2" << YAML::Value
                 << calib_est_param.phi_2;

  std::ofstream result_stream(curr_result_path + "/calib_result.yaml");
  std::ofstream newest_result_stream(result_path + "/newest_calib_result.yaml");
  result_stream << result_emitter.c_str();
  newest_result_stream << result_emitter.c_str();
  result_stream.close();
  newest_result_stream.close();

  Eigen::Matrix3d calib_R_x_phi_1 =
      Eigen::AngleAxisd(calib_est_param.phi_1, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d calib_R_z_theta_2 =
      Eigen::AngleAxisd(calib_est_param.theta_2, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d calib_R_x_phi_2 =
      Eigen::AngleAxisd(calib_est_param.phi_2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Vector3d calib_t_2(calib_est_param.a_1, 0.0, calib_est_param.d_1);
  Eigen::Vector3d calib_t_1(calib_est_param.a_2, 0.0, calib_est_param.d_2);

  Eigen::Matrix3d uncalib_R_x_phi_1 =
      Eigen::AngleAxisd(uncalib_est_param.phi_1, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d uncalib_R_z_theta_2 =
      Eigen::AngleAxisd(uncalib_est_param.theta_2, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Matrix3d uncalib_R_x_phi_2 =
      Eigen::AngleAxisd(uncalib_est_param.phi_2, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Vector3d uncalib_t_2(
      uncalib_est_param.a_1, 0.0, uncalib_est_param.d_1);
  Eigen::Vector3d uncalib_t_1(
      uncalib_est_param.a_2, 0.0, uncalib_est_param.d_2);

  for (size_t i = 0; i < group_point_array.size(); ++i) {
    pcl::PointCloud<PointType>::Ptr calib_cloud(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr uncalib_cloud(
        new pcl::PointCloud<PointType>());
    Eigen::Matrix3d R_W_B = R_W_B_array_[i];
    for (const auto& point_ptr : group_point_array[i]) {
      Eigen::Matrix3d R_z_theta_1 =
          Eigen::AngleAxisd(point_ptr->angle, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();

      Eigen::Matrix3d calib_R_B_L =
          R_z_theta_1 * calib_R_x_phi_1 * calib_R_z_theta_2 * calib_R_x_phi_2;
      Eigen::Vector3d calib_t_B_L =
          R_z_theta_1 * calib_R_x_phi_1 * calib_R_z_theta_2 * calib_t_1 +
          R_z_theta_1 * calib_t_2;

      Eigen::Vector3d calib_p_B =
          R_W_B * calib_R_B_L * point_ptr->p + R_W_B * calib_t_B_L;

      PointType calib_pt;
      calib_pt.x = calib_p_B.x();
      calib_pt.y = calib_p_B.y();
      calib_pt.z = calib_p_B.z();
      calib_cloud->push_back(calib_pt);

      Eigen::Matrix3d uncalib_R_B_L = R_z_theta_1 * uncalib_R_x_phi_1 *
                                      uncalib_R_z_theta_2 * uncalib_R_x_phi_2;
      Eigen::Vector3d uncalib_t_B_L =
          R_z_theta_1 * uncalib_R_x_phi_1 * uncalib_R_z_theta_2 * uncalib_t_1 +
          R_z_theta_1 * uncalib_t_2;

      Eigen::Vector3d uncalib_p_B =
          R_W_B * uncalib_R_B_L * point_ptr->p + R_W_B * uncalib_t_B_L;

      PointType uncalib_pt;
      uncalib_pt.x = uncalib_p_B.x();
      uncalib_pt.y = uncalib_p_B.y();
      uncalib_pt.z = uncalib_p_B.z();
      uncalib_cloud->push_back(uncalib_pt);
    }

    std::string calib_cloud_path =
        curr_result_path + "/calib_" + std::to_string(i) + ".pcd";
    std::string uncalib_cloud_path =
        curr_result_path + "/uncalib_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFileASCII(calib_cloud_path, *calib_cloud);
    pcl::io::savePCDFileASCII(uncalib_cloud_path, *uncalib_cloud);
  }
}

void RotationLidarCalibration::LoadExtrinsicParam(const std::string& path,
                                                  ExtrinsicParam& ext_param) {
  YAML::Node node = YAML::LoadFile(path);

  ext_param.d_1 = node["d_1"].as<double>();
  ext_param.a_1 = node["a_1"].as<double>();
  ext_param.phi_1 = node["phi_1"].as<double>();
  ext_param.theta_2 = node["theta_2"].as<double>();
  ext_param.d_2 = node["d_2"].as<double>();
  ext_param.a_2 = node["a_2"].as<double>();
  ext_param.phi_2 = node["phi_2"].as<double>();

  LOG(INFO) << "[LoadExtrinsicParam]" << std::endl
            << "\td_1: " << ext_param.d_1 << std::endl
            << "\ta_1: " << ext_param.a_1 << std::endl
            << "\tphi_1: " << ext_param.phi_1 << std::endl
            << "\ttheta_2: " << ext_param.theta_2 << std::endl
            << "\td_2: " << ext_param.d_2 << std::endl
            << "\ta_2: " << ext_param.a_2 << std::endl
            << "\tphi_2: " << ext_param.phi_2;
}
