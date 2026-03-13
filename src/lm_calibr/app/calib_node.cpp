#include <development_tools/tools.h>

#include "lm_calibr/rotation_lidar_calibration.h"

dev_tools::Timer timer;

int main(int argc, char** argv) {
  ros::init(argc, argv, "calib_node");
  ros::NodeHandle nh;

  dev_tools::Logger::Config logger_config;
  logger_config.log_prefix = false;
  std::string package_path = ros::package::getPath("lm_calibr");
  dev_tools::Logger logger(argc, argv, package_path, logger_config);

  std::string cloud_topic;
  nh.param<std::string>("/lidar_topic", cloud_topic, "/cloud");
  std::string encoder_topic;
  nh.param<std::string>("/encoder_topic", encoder_topic, "/encoder");
  LOG(INFO) << "encoder_topic: " << encoder_topic;
  LOG(INFO) << "cloud_topic: " << cloud_topic;

  int max_iter;
  nh.param<int>("max_iter", max_iter, 50);
  double downsample_size;
  nh.param<double>("downsample_size", downsample_size, 0.1);
  double max_voxel_size;
  nh.param<double>("max_voxel_size", max_voxel_size, 1.0);
  int max_layer;
  nh.param<int>("max_layer", max_layer, 2);
  double min_range;
  nh.param<double>("min_range", min_range, 0.5);
  double max_range;
  nh.param<double>("max_range", max_range, 50.0);
  std::vector<float> eigen_threshold;
  nh.param<std::vector<float>>(
      "eigen_threshold", eigen_threshold, std::vector<float>());
  if (eigen_threshold.size() != static_cast<size_t>(max_layer + 1)) {
    LOG(ERROR) << "eigen_threshold.size() != max_layer + 1";
    exit(0);
  }
  double rosbag_skip;
  nh.param<double>("rosbag_skip", rosbag_skip, 0.0);
  double angle_threshold;
  nh.param<double>("angle_threshold", angle_threshold, 0.0);

  int DH_type = 0;
  nh.param<int>("DH_type", DH_type, 0);
  LOG(INFO) << "DH_type: " << DH_type;

  RotationLidarCalibration::ExtrinsicParam init_ext;
  nh.param<double>("init_extrinsic/d_1", init_ext.d_1, 0.0);
  nh.param<double>("init_extrinsic/a_1", init_ext.a_1, 0.0);
  nh.param<double>("init_extrinsic/phi_1", init_ext.phi_1, 0.0);
  nh.param<double>("init_extrinsic/theta_2", init_ext.theta_2, 0.0);
  nh.param<double>("init_extrinsic/d_2", init_ext.d_2, 0.0);
  nh.param<double>("init_extrinsic/a_2", init_ext.a_2, 0.0);
  nh.param<double>("init_extrinsic/phi_2", init_ext.phi_2, 0.0);
  LOG(INFO) << "init_extrinsic" << std::endl
            << "\td_1: " << init_ext.d_1 << std::endl
            << "\ta_1: " << init_ext.a_1 << std::endl
            << "\tphi_1: " << init_ext.phi_1 << std::endl
            << "\ttheta_2: " << init_ext.theta_2 << std::endl
            << "\td_2: " << init_ext.d_2 << std::endl
            << "\ta_2: " << init_ext.a_2 << std::endl
            << "\tphi_2: " << init_ext.phi_2;

  std::vector<std::string> bag_path_array;
  nh.param<std::vector<std::string>>(
      "bag_path", bag_path_array, std::vector<std::string>());
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

  RotationLidarCalibration::ExtrinsicParam est_ext = init_ext;

  // load PCD
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

          if (iter > 2 && calib.first_iter_convergence_) {
            break;
          }
        }
      },
      "total time");

  // save results

  LOG(INFO) << "begin save result" << std::endl;
  calib.SaveResult(result_path, group_point_array, est_ext, init_ext);

  timer.PrintAll();

  return 0;
}