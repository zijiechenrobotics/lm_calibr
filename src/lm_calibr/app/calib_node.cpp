#include <development_tools/tools.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lm_calibr/rotation_lidar_calibration.h"

dev_tools::Timer timer;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("calib_node");

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

  std::string cloud_topic;
  node->declare_parameter<std::string>("lidar_topic", "/cloud");
  node->get_parameter("lidar_topic", cloud_topic);

  std::string encoder_topic;
  node->declare_parameter<std::string>("encoder_topic", "/encoder");
  node->get_parameter("encoder_topic", encoder_topic);

  LOG(INFO) << "encoder_topic: " << encoder_topic;
  LOG(INFO) << "cloud_topic: " << cloud_topic;

  int max_iter;
  node->declare_parameter("max_iter", 50);
  node->get_parameter("max_iter", max_iter);

  double downsample_size;
  node->declare_parameter("downsample_size", 0.1);
  node->get_parameter("downsample_size", downsample_size);

  double max_voxel_size;
  node->declare_parameter("max_voxel_size", 1.0);
  node->get_parameter("max_voxel_size", max_voxel_size);

  int max_layer;
  node->declare_parameter("max_layer", 2);
  node->get_parameter("max_layer", max_layer);

  double min_range;
  node->declare_parameter("min_range", 0.5);
  node->get_parameter("min_range", min_range);

  double max_range;
  node->declare_parameter("max_range", 50.0);
  node->get_parameter("max_range", max_range);

  std::vector<double> eigen_threshold;
  node->declare_parameter<std::vector<double>>("eigen_threshold",
                                               std::vector<double>());
  node->get_parameter("eigen_threshold", eigen_threshold);

  if (eigen_threshold.size() != static_cast<size_t>(max_layer + 1)) {
    LOG(ERROR) << "eigen_threshold.size() != max_layer + 1";
    exit(0);
  }

  double rosbag_skip;
  node->declare_parameter("rosbag_skip", 0.0);
  node->get_parameter("rosbag_skip", rosbag_skip);

  double angle_threshold;
  node->declare_parameter("angle_threshold", 0.0);
  node->get_parameter("angle_threshold", angle_threshold);

  int DH_type = 0;
  node->declare_parameter("DH_type", 0);
  node->get_parameter("DH_type", DH_type);
  LOG(INFO) << "DH_type: " << DH_type;

  RotationLidarCalibration::ExtrinsicParam init_ext;
  node->declare_parameter("init_extrinsic.d_1", 0.0);
  node->declare_parameter("init_extrinsic.a_1", 0.0);
  node->declare_parameter("init_extrinsic.phi_1", 0.0);
  node->declare_parameter("init_extrinsic.theta_2", 0.0);
  node->declare_parameter("init_extrinsic.d_2", 0.0);
  node->declare_parameter("init_extrinsic.a_2", 0.0);
  node->declare_parameter("init_extrinsic.phi_2", 0.0);

  node->get_parameter("init_extrinsic.d_1", init_ext.d_1);
  node->get_parameter("init_extrinsic.a_1", init_ext.a_1);
  node->get_parameter("init_extrinsic.phi_1", init_ext.phi_1);
  node->get_parameter("init_extrinsic.theta_2", init_ext.theta_2);
  node->get_parameter("init_extrinsic.d_2", init_ext.d_2);
  node->get_parameter("init_extrinsic.a_2", init_ext.a_2);
  node->get_parameter("init_extrinsic.phi_2", init_ext.phi_2);

  LOG(INFO) << "init_extrinsic" << std::endl
            << "\td_1: " << init_ext.d_1 << std::endl
            << "\ta_1: " << init_ext.a_1 << std::endl
            << "\tphi_1: " << init_ext.phi_1 << std::endl
            << "\ttheta_2: " << init_ext.theta_2 << std::endl
            << "\td_2: " << init_ext.d_2 << std::endl
            << "\ta_2: " << init_ext.a_2 << std::endl
            << "\tphi_2: " << init_ext.phi_2;

  std::vector<std::string> bag_path_array;
  node->declare_parameter<std::vector<std::string>>("bag_path",
                                                    std::vector<std::string>());
  node->get_parameter("bag_path", bag_path_array);

  if (bag_path_array.empty()) {
    LOG(ERROR) << "bag_path_array is empty";
    exit(0);
  }

  LOG(INFO) << "bag_path_array";
  for (const auto path : bag_path_array) {
    LOG(INFO) << "\t" << path;
  }

  RotationLidarCalibration::Config config;
  config.max_iter = max_iter;
  config.downsample_size = downsample_size;
  config.max_voxel_size = max_voxel_size;
  config.max_layer = max_layer;
  config.min_range = min_range;
  config.max_range = max_range;
  config.eigen_threshold = eigen_threshold;
  config.DH_type = DH_type;
  config.rosbag_skip = rosbag_skip;
  config.angle_threshold = angle_threshold;

  RotationLidarCalibration calib(config);

  std::string database_path = package_path + "/result/calib_database";
  std::string result_path = package_path + "/result/calib_results";

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

  RotationLidarCalibration::ExtrinsicParam est_ext = init_ext;

  LOG(INFO) << "begin load database" << std::endl;
  std::vector<std::vector<std::shared_ptr<Point>>> group_point_array;
  calib.LoadDatabase(database_path, group_point_array);

  timer.Evaluate(
      [&]() {
        for (size_t iter = 0; iter < 10; ++iter) {
          if (iter < 2) {
            calib.config_.max_voxel_size = 1.0;
          } else if (2 <= iter && iter < 4) {
            calib.config_.max_voxel_size = 0.5;
          } else {
            calib.config_.max_voxel_size = 0.25;
          }

          LOG(INFO) << "outer iter: " << iter
                    << ", voxel_size: " << calib.config_.max_voxel_size;

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

          timer.Evaluate([&]() { calib.OutlierRemove(planes_array, est_ext); },
                         "remove outiler");

          timer.Evaluate([&]() { calib.Optimization(planes_array, est_ext); },
                         "optimization");

          if (iter > 2 && calib.first_iter_convergence_) {
            break;
          }
        }
      },
      "total time");

  LOG(INFO) << "begin save result" << std::endl;
  calib.SaveResult(result_path, group_point_array, est_ext, init_ext);

  timer.PrintAll();

  rclcpp::shutdown();
  return 0;
}