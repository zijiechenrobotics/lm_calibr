#include <development_tools/tools.h>

#include <filesystem>
#include <memory>
#include <random>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lm_calibr/rotation_lidar_calibration.h"
#include "rclcpp/rclcpp.hpp"

dev_tools::Timer timer;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_calib_node");

  std::string package_path =
      ament_index_cpp::get_package_share_directory("lm_calibr");
  dev_tools::Logger::Config logger_config;
  logger_config.log_prefix = false;
  auto nonros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> logger_argv;
  logger_argv.reserve(nonros_args.size());
  for (auto& arg : nonros_args) {
    logger_argv.push_back(arg.data());
  }
  dev_tools::Logger logger(static_cast<int>(logger_argv.size()),
                           logger_argv.data(),
                           package_path,
                           logger_config);

  node->declare_parameter<std::string>("lidar_topic", "/cloud");
  node->declare_parameter<std::string>("encoder_topic", "/encoder");
  node->declare_parameter<int>("random_seed", 0);
  node->declare_parameter<int>("max_iter", 50);
  node->declare_parameter<double>("downsample_size", 0.1);
  node->declare_parameter<double>("max_voxel_size", 1.0);
  node->declare_parameter<int>("max_layer", 2);
  node->declare_parameter<std::vector<double>>("eigen_threshold");
  node->declare_parameter<double>("rosbag_skip", 0.0);
  node->declare_parameter<double>("angle_threshold", 0.0);
  node->declare_parameter<std::vector<std::string>>("bag_path");
  node->declare_parameter<int>("DH_type", 0);
  node->declare_parameter<double>("gt_extrinsic.d_1", 0.0);
  node->declare_parameter<double>("gt_extrinsic.a_1", 0.0);
  node->declare_parameter<double>("gt_extrinsic.phi_1", 0.0);
  node->declare_parameter<double>("gt_extrinsic.theta_2", 0.0);
  node->declare_parameter<double>("gt_extrinsic.d_2", 0.0);
  node->declare_parameter<double>("gt_extrinsic.a_2", 0.0);
  node->declare_parameter<double>("gt_extrinsic.phi_2", 0.0);

  std::string cloud_topic = node->get_parameter("lidar_topic").as_string();
  std::string encoder_topic = node->get_parameter("encoder_topic").as_string();
  LOG(INFO) << "encoder_topic: " << encoder_topic << std::endl;
  LOG(INFO) << "cloud_topic: " << cloud_topic << std::endl;

  int tseed = node->get_parameter("random_seed").as_int();

  int max_iter = node->get_parameter("max_iter").as_int();
  double downsample_size = node->get_parameter("downsample_size").as_double();
  double max_voxel_size = node->get_parameter("max_voxel_size").as_double();
  int max_layer = node->get_parameter("max_layer").as_int();
  ;
  std::vector<double> eigen_threshold =
      node->get_parameter("eigen_threshold").as_double_array();
  if (eigen_threshold.size() != static_cast<size_t>(max_layer + 1)) {
    LOG(ERROR) << "eigen_threshold.size() != max_layer + 1" << std::endl;
    exit(0);
  }
  double rosbag_skip = node->get_parameter("rosbag_skip").as_double();
  double angle_threshold = node->get_parameter("angle_threshold").as_double();

  std::vector<std::string> bag_path_array =
      node->get_parameter("bag_path").as_string_array();
  if (bag_path_array.empty()) {
    LOG(ERROR) << "bag_path_array is empty, force exit" << std::endl;
    exit(0);
  }
  LOG(INFO) << "bag_path_array" << std::endl;
  for (const auto path : bag_path_array) {
    LOG(INFO) << "\t" << path << std::endl;
  }

  int DH_type = node->get_parameter("DH_type").as_int();
  LOG(INFO) << "DH_type: " << DH_type;

  RotationLidarCalibration::ExtrinsicParam gt_ext;
  node->get_parameter("gt_extrinsic.d_1", gt_ext.d_1);
  node->get_parameter("gt_extrinsic.a_1", gt_ext.a_1);
  node->get_parameter("gt_extrinsic.phi_1", gt_ext.phi_1);
  node->get_parameter("gt_extrinsic.theta_2", gt_ext.theta_2);
  node->get_parameter("gt_extrinsic.d_2", gt_ext.d_2);
  node->get_parameter("gt_extrinsic.a_2", gt_ext.a_2);
  node->get_parameter("gt_extrinsic.phi_2", gt_ext.phi_2);
  LOG(INFO) << "gt_extrinsic" << std::endl
            << "\td_1: " << gt_ext.d_1 << std::endl
            << "\ta_1: " << gt_ext.a_1 << std::endl
            << "\tphi_1: " << gt_ext.phi_1 << std::endl
            << "\ttheta_2: " << gt_ext.theta_2 << std::endl
            << "\td_2: " << gt_ext.d_2 << std::endl
            << "\ta_2: " << gt_ext.a_2 << std::endl
            << "\tphi_2: " << gt_ext.phi_2;

  std::default_random_engine e;
  e.seed(tseed);
  // std::normal_distribution<double> rand_rot(0, 2 / 57.3);
  std::normal_distribution<double> rand_rot(0, 0.2);
  std::normal_distribution<double> rand_trans(0, 0.1);
  RotationLidarCalibration::ExtrinsicParam est_ext = gt_ext;
  if (DH_type == 0) {
    double phi_1_error = rand_rot(e);
    // double phi_1_error = 0.2;
    est_ext.phi_1 += phi_1_error;
    // est_ext.phi_1 += 0.2;
    if (est_ext.phi_1 < -M_PI) {
      est_ext.phi_1 += M_PI * 2.0;
    } else if (est_ext.phi_1 > M_PI) {
      est_ext.phi_1 -= M_PI * 2.0;
    }

    double theta_2_error = rand_rot(e);
    // double theta_2_error = -0.2;
    est_ext.theta_2 += theta_2_error;
    if (est_ext.theta_2 < -M_PI) {
      est_ext.theta_2 += M_PI * 2.0;
    } else if (est_ext.theta_2 > M_PI) {
      est_ext.theta_2 -= M_PI * 2.0;
    }

    double a_1_error = rand_trans(e);
    est_ext.a_1 += a_1_error;

    double d_2_error = rand_trans(e);
    est_ext.d_2 += d_2_error;

    LOG(INFO) << "phi_1_error: " << phi_1_error / M_PI * 180.0 << " rad"
              << ", theta_2_error: " << theta_2_error / M_PI * 180.0 << " rad"
              << ", a_1: " << a_1_error << " m"
              << ", d_2: " << d_2_error << " m";
  } else if (DH_type == 1) {
    double theta_2_error = rand_rot(e);
    est_ext.theta_2 += theta_2_error;
    if (est_ext.theta_2 < -M_PI) {
      est_ext.theta_2 += M_PI * 2.0;
    } else if (est_ext.theta_2 > M_PI) {
      est_ext.theta_2 -= M_PI * 2.0;
    }

    double phi_2_error = rand_rot(e);
    est_ext.phi_2 += phi_2_error;
    if (est_ext.phi_2 < -M_PI) {
      est_ext.phi_2 += M_PI * 2.0;
    } else if (est_ext.phi_2 > M_PI) {
      est_ext.phi_2 -= M_PI * 2.0;
    }

    double a_2_error = rand_trans(e);
    est_ext.a_2 += a_2_error;

    double d_2_error = rand_trans(e);
    est_ext.d_2 += d_2_error;

    LOG(INFO) << "phi_1_error: " << theta_2_error / M_PI * 180.0 << " rad"
              << ", theta_2_error: " << phi_2_error / M_PI * 180.0 << " rad"
              << ", a_1_error: " << a_2_error * 1000.0 << " mm"
              << ", d_2_error: " << d_2_error * 1000.0 << " mm";
  } else {
    LOG(ERROR) << "error DH_type: " << DH_type << ", force exit";
    exit(0);
  }

  RotationLidarCalibration::Config config;
  config.max_iter = max_iter;
  config.downsample_size = downsample_size;
  config.max_voxel_size = max_voxel_size;
  config.max_layer = max_layer;
  config.eigen_threshold = eigen_threshold;
  config.DH_type = DH_type;
  config.angle_threshold = angle_threshold;
  config.rosbag_skip = rosbag_skip;
  RotationLidarCalibration calib(config);

  std::string database_path = package_path + "/result/calib_database";
  std::string result_path = package_path + "/result/calib_results";

  // forcibly delete the previous preprocessing and preprocess it again
  std::filesystem::remove_all(database_path);
  std::filesystem::remove_all(result_path);
  std::filesystem::create_directories(database_path);
  if (!calib.ProcessRosbags(bag_path_array,
                            cloud_topic,
                            LidarType::LIVOX,
                            encoder_topic,
                            database_path)) {
    LOG(ERROR) << "fail to process rosbag, force exit" << std::endl;
    exit(0);
  }

  RotationLidarCalibration::ExtrinsicParam unest_ext = est_ext;

  LOG(INFO) << "before calib" << std::endl;
  LOG(INFO) << "[gt  ] a_1: " << gt_ext.a_1 << ", phi_1: " << gt_ext.phi_1
            << ", theta_2: " << gt_ext.theta_2 << ", d_2: " << gt_ext.d_2
            << ", a_2: " << gt_ext.a_2 << ", phi_2: " << gt_ext.phi_2;
  LOG(INFO) << "[est ] a_1: " << est_ext.a_1 << ", phi_1: " << est_ext.phi_1
            << ", theta_2: " << est_ext.theta_2 << ", d_2: " << est_ext.d_2
            << ", a_2: " << est_ext.a_2 << ", phi_2: " << est_ext.phi_2;

  // load PCD
  LOG(INFO) << "begin load database" << std::endl;
  std::vector<std::vector<std::shared_ptr<Point>>> group_point_array;
  calib.LoadDatabase(database_path, group_point_array);

  timer.Evaluate(
      [&]() {
        for (size_t iter = 0; iter < 10; ++iter) {
          LOG(INFO) << "outer iter: " << iter;

          // set the size of voxel_map base on the number of iterations
          if (iter < 2) {
            calib.config_.max_voxel_size = 1.0;
          } else if (2 <= iter && iter < 4) {
            calib.config_.max_voxel_size = 0.5;
          } else {
            calib.config_.max_voxel_size = 0.25;
          }

          // build voxel_map
          std::vector<std::shared_ptr<Plane>> planes_array;
          timer.Evaluate(
              [&]() {
                if (!calib.BuildVoxelMap(
                        group_point_array, est_ext, planes_array)) {
                  LOG(ERROR)
                      << "fail to build voxel map, force exit" << std::endl;
                  exit(0);
                }
              },
              "bulid voxel map");

          // outlier elimination
          timer.Evaluate([&]() { calib.OutlierRemove(planes_array, est_ext); },
                         "remove outiler");
          // optimization
          timer.Evaluate([&]() { calib.Optimization(planes_array, est_ext); },
                         "optimization");

          LOG(INFO) << "after calib" << std::endl;
          LOG(INFO) << "[gt  ] a_1: " << gt_ext.a_1
                    << ", phi_1: " << gt_ext.phi_1
                    << ", theta_2: " << gt_ext.theta_2
                    << ", d_2: " << gt_ext.d_2 << ", a_2: " << gt_ext.a_2
                    << ", phi_2: " << gt_ext.phi_2;
          LOG(INFO) << "[est ] a_1: " << est_ext.a_1
                    << ", phi_1: " << est_ext.phi_1
                    << ", theta_2: " << est_ext.theta_2
                    << ", d_2: " << est_ext.d_2 << ", a_2: " << est_ext.a_2
                    << ", phi_2: " << est_ext.phi_2;
          LOG(INFO) << "[diff] a_1: " << std::abs(gt_ext.a_1 - est_ext.a_1)
                    << " m"
                    << ", phi_1: "
                    << std::abs(gt_ext.phi_1 - est_ext.phi_1) / M_PI * 180.0
                    << " deg"
                    << ", theta_2: "
                    << std::abs(gt_ext.theta_2 - est_ext.theta_2) / M_PI * 180.0
                    << " deg"
                    << ", d_2: " << std::abs(gt_ext.d_2 - est_ext.d_2) * 1000.0
                    << " mm"
                    << ", a_2: " << std::abs(gt_ext.a_2 - est_ext.a_2) * 1000.0
                    << " mm"
                    << ", phi_2: "
                    << std::abs(gt_ext.phi_2 - est_ext.phi_2) / M_PI * 180.0
                    << " deg";

          if (iter > 2 && calib.first_iter_convergence_) {
            break;
          }
        }
      },
      "total time");

  // save results

  LOG(INFO) << "begin save result" << std::endl;
  calib.SaveResult(result_path, group_point_array, est_ext, unest_ext);

  LOG(INFO) << "after calib" << std::endl;
  LOG(INFO) << "[gt  ] a_1: " << gt_ext.a_1 << ", phi_1: " << gt_ext.phi_1
            << ", theta_2: " << gt_ext.theta_2 << ", d_2: " << gt_ext.d_2
            << ", a_2: " << gt_ext.a_2 << ", phi_2: " << gt_ext.phi_2;
  LOG(INFO) << "[est ] a_1: " << est_ext.a_1 << ", phi_1: " << est_ext.phi_1
            << ", theta_2: " << est_ext.theta_2 << ", d_2: " << est_ext.d_2
            << ", a_2: " << est_ext.a_2 << ", phi_2: " << est_ext.phi_2;
  LOG(INFO) << "[diff] a_1: " << std::abs(gt_ext.a_1 - est_ext.a_1) * 1000.0
            << " mm"
            << ", phi_1: "
            << std::abs(gt_ext.phi_1 - est_ext.phi_1) / M_PI * 180.0 << " deg"
            << ", theta_2: "
            << std::abs(gt_ext.theta_2 - est_ext.theta_2) / M_PI * 180.0
            << " deg"
            << ", d_2: " << std::abs(gt_ext.d_2 - est_ext.d_2) * 1000.0 << " mm"
            << ", a_2: " << std::abs(gt_ext.a_2 - est_ext.a_2) * 1000.0 << " mm"
            << ", phi_2: "
            << std::abs(gt_ext.phi_2 - est_ext.phi_2) / M_PI * 180.0 << " deg";

  LOG(INFO) << "finsh" << std::endl;

  timer.PrintAll();

  return 0;
}