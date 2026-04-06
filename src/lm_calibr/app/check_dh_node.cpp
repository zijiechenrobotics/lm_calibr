#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lm_calibr/rotation_lidar_calibration.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_sim_rosbag_node");

  std::string package_path =
      ament_index_cpp::get_package_share_directory("lm_calibr");

  double d_1 = 0.0;
  double a_1 = 0.0;
  double phi_1 = 0.0;
  double theta_2 = 0.0;
  double d_2 = 0.0;
  double a_2 = 0.0;
  double phi_2 = 0.0;

  node->declare_parameter("init_extrinsic.d_1", 0.0);
  node->declare_parameter("init_extrinsic.a_1", 0.0);
  node->declare_parameter("init_extrinsic.phi_1", 0.0);
  node->declare_parameter("init_extrinsic.theta_2", 0.0);
  node->declare_parameter("init_extrinsic.d_2", 0.0);
  node->declare_parameter("init_extrinsic.a_2", 0.0);
  node->declare_parameter("init_extrinsic.phi_2", 0.0);

  node->get_parameter("init_extrinsic.d_1", d_1);
  node->get_parameter("init_extrinsic.a_1", a_1);
  node->get_parameter("init_extrinsic.phi_1", phi_1);
  node->get_parameter("init_extrinsic.theta_2", theta_2);
  node->get_parameter("init_extrinsic.d_2", d_2);
  node->get_parameter("init_extrinsic.a_2", a_2);
  node->get_parameter("init_extrinsic.phi_2", phi_2);

  std::vector<std::string> tmp_bag_path_array;
  node->declare_parameter<std::vector<std::string>>("bag_path",
                                                    std::vector<std::string>());
  node->get_parameter("bag_path", tmp_bag_path_array);

  if (tmp_bag_path_array.empty()) {
    LOG(ERROR) << "tmp_bag_path_array is empty";
    exit(0);
  }

  std::string rosbag_path = tmp_bag_path_array[0];

  double rosbag_skip = 0.0;
  node->declare_parameter("rosbag_skip", 0.0);
  node->get_parameter("rosbag_skip", rosbag_skip);

  double angle_threshold = 0.0;
  node->declare_parameter("angle_threshold", 0.0);
  node->get_parameter("angle_threshold", angle_threshold);

  pcl::PointCloud<PointType>::Ptr encoder_map(new pcl::PointCloud<PointType>());

  Eigen::Matrix3d R_x_phi_1 =
      Eigen::AngleAxisd(phi_1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Matrix3d R_z_theta_2 =
      Eigen::AngleAxisd(theta_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d R_x_phi_2 =
      Eigen::AngleAxisd(phi_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Vector3d t_2(a_1, 0.0, d_1);
  Eigen::Vector3d t_1(a_2, 0.0, d_2);

  pcl::PointCloud<PointType>::Ptr origin_map(new pcl::PointCloud<PointType>());

  RotationLidarCalibration::Config calib_config;
  calib_config.downsample_size = 0.1;
  calib_config.rosbag_skip = rosbag_skip;
  RotationLidarCalibration calib(calib_config);

  std::string database_path = package_path + "/result/calib_database";

  // preprocessed cloud
  std::vector<std::string> bag_path_array = {rosbag_path};
  std::filesystem::remove_all(database_path);
  std::filesystem::create_directories(database_path);

  calib.ProcessRosbags(bag_path_array,
                       "/livox/lidar",
                       LidarType::LIVOX,
                       "/joint_states",
                       database_path);

  // load preprocessed cloud
  std::vector<std::vector<std::shared_ptr<Point>>> group_point_array;
  calib.LoadDatabase(database_path, group_point_array);

  // from LiDAR frame to base frame
  for (const auto& point_ptr : group_point_array[0]) {
    Eigen::Matrix3d R_z_theta_1 =
        Eigen::AngleAxisd(point_ptr->angle, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    Eigen::Matrix3d R_B_L = R_z_theta_1 * R_x_phi_1 * R_z_theta_2 * R_x_phi_2;
    Eigen::Vector3d t_B_L =
        R_z_theta_1 * R_x_phi_1 * R_z_theta_2 * t_1 + R_z_theta_1 * t_2;

    Eigen::Vector3d p_B = R_B_L * point_ptr->p + t_B_L;

    PointType pt;
    pt.x = p_B.x();
    pt.y = p_B.y();
    pt.z = p_B.z();
    encoder_map->push_back(pt);
  }

  std::string encoder_map_path = package_path + "/result/encoder_cloud.pcd";
  pcl::io::savePCDFileASCII(encoder_map_path, *encoder_map);
  std::cout << "save in: " << encoder_map_path << std::endl;

  rclcpp::shutdown();
  return 0;
}