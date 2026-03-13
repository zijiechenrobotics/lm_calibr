#include <development_tools/tools.h>
#include <ros/ros.h>

#include <mutex>

#include "lm_calibr/rotation_lidar_calibration.h"
#include "signal.h"

Eigen::Matrix3d R_x_phi_1 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d R_z_theta_2 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d R_x_phi_2 = Eigen::Matrix3d::Identity();
Eigen::Vector3d t_1 = Eigen::Vector3d::Zero();
Eigen::Vector3d t_2 = Eigen::Vector3d::Zero();
double time_offset = 0.0;

struct LivoxMsg {
  double timestamp = 0.0;
  livox_ros_driver::CustomMsg::Ptr livox_cloud_ptr = nullptr;
};

std::deque<std::shared_ptr<LivoxMsg>> cloud_array;
std::deque<std::shared_ptr<EncoderMsg>> encoder_array;

std::mutex array_mutex;

double lidar_sample_time = 1.0 / 10.0;

ros::Publisher calibrated_cloud_pub;
ros::Publisher calibrated_livox_pub;

void LivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg_ptr) {
  static double last_timestamp = 0.0;
  std::shared_ptr<LivoxMsg> cloud_msg_ptr(new LivoxMsg());
  cloud_msg_ptr->livox_cloud_ptr.reset(
      new livox_ros_driver::CustomMsg(*msg_ptr));
  cloud_msg_ptr->timestamp = msg_ptr->header.stamp.toSec();

  std::sort(cloud_msg_ptr->livox_cloud_ptr->points.begin(),
            cloud_msg_ptr->livox_cloud_ptr->points.end(),
            [](const auto& a, const auto& b) {
              return a.offset_time < b.offset_time;
            });

  {
    std::lock_guard<std::mutex> lock(array_mutex);

    if (cloud_msg_ptr->timestamp < last_timestamp) {
      LOG(WARNING) << "livox loop back, clear buffer" << std::endl;
      cloud_array.clear();
    }

    cloud_array.push_back(cloud_msg_ptr);
    last_timestamp = cloud_msg_ptr->timestamp;
  }
}

void EncoderCallback(const sensor_msgs::JointState::ConstPtr& msg_ptr) {
  static double last_timestamp = 0.0;
  std::shared_ptr<EncoderMsg> encoder_msg_ptr(new EncoderMsg());
  encoder_msg_ptr->angle = msg_ptr->position[0];
  encoder_msg_ptr->angle_vel = msg_ptr->velocity[0];
  encoder_msg_ptr->timestamp = msg_ptr->header.stamp.toSec() + time_offset;

  {
    std::lock_guard<std::mutex> lock(array_mutex);

    if (encoder_msg_ptr->timestamp < last_timestamp) {
      LOG(WARNING) << "encoder loop back, clear buffer" << std::endl;
      encoder_array.clear();
    }

    encoder_array.push_back(encoder_msg_ptr);
    last_timestamp = encoder_msg_ptr->timestamp;
  }
}

bool SyncMsg() {
  if (cloud_array.empty() || encoder_array.size() < 2) {
    return false;
  }

  if (encoder_array.back()->timestamp <
      (cloud_array.front()->timestamp + lidar_sample_time)) {
    return false;
  }

  if (cloud_array.front()->timestamp < encoder_array.front()->timestamp) {
    std::lock_guard<std::mutex> lock(array_mutex);
    cloud_array.pop_front();

    return false;
  }

  return true;
}

void Process() {
  std::lock_guard<std::mutex> lock(array_mutex);

  auto cloud_msg_ptr = cloud_array.front();
  double cloud_msg_timestamp = cloud_msg_ptr->timestamp;

  while (encoder_array[1]->timestamp < cloud_msg_timestamp) {
    encoder_array.pop_front();
  }

  pcl::PointCloud<PointType>::Ptr calibrated_cloud_ptr(
      new pcl::PointCloud<PointType>());
  livox_ros_driver::CustomMsg calibrated_livox_msg;

  size_t i = 0;
  PointType pt;
  livox_ros_driver::CustomPoint calibrated_livox_pt;
  for (auto& livox_pt : cloud_msg_ptr->livox_cloud_ptr->points) {
    double pt_time = cloud_msg_timestamp + livox_pt.offset_time * 1e-9;

    if (encoder_array[i]->timestamp <= pt_time &&
        pt_time <= encoder_array[i + 1]->timestamp) {
      double angle = RotationLidarCalibration::AngleInterpolate(
          encoder_array[i]->angle,
          encoder_array[i]->timestamp,
          encoder_array[i + 1]->angle,
          encoder_array[i + 1]->timestamp,
          pt_time);

      Eigen::Matrix3d R_z_theta_1 =
          Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      Eigen::Matrix3d R_B_L = R_z_theta_1 * R_x_phi_1 * R_z_theta_2 * R_x_phi_2;
      Eigen::Vector3d t_B_L =
          R_z_theta_1 * R_x_phi_1 * R_z_theta_2 * t_1 + R_z_theta_1 * t_2;
      Eigen::Vector3d p_L(livox_pt.x, livox_pt.y, livox_pt.z);
      Eigen::Vector3d p_B = R_B_L * p_L + t_B_L;

      pt.x = p_B.x();
      pt.y = p_B.y();
      pt.z = p_B.z();
      pt.intensity = livox_pt.reflectivity;
      calibrated_cloud_ptr->push_back(pt);

      calibrated_livox_pt = livox_pt;
      calibrated_livox_pt.x = p_B.x();
      calibrated_livox_pt.y = p_B.y();
      calibrated_livox_pt.z = p_B.z();
      calibrated_livox_msg.points.push_back(calibrated_livox_pt);
    } else if (pt_time > encoder_array[i + 1]->timestamp) {
      i++;
      if (i + 1 >= encoder_array.size()) {
        break;
      }
    }
  }

  LOG(INFO) << "pulish: " << calibrated_cloud_ptr->points.size() << std::endl;

  ros::Time timestamp(cloud_msg_timestamp);

  sensor_msgs::PointCloud2 calibrated_cloud_msg;
  pcl::toROSMsg(*calibrated_cloud_ptr, calibrated_cloud_msg);
  calibrated_cloud_msg.header.frame_id = "world";
  calibrated_cloud_msg.header.stamp = timestamp;
  calibrated_cloud_pub.publish(calibrated_cloud_msg);

  calibrated_livox_msg.header = cloud_msg_ptr->livox_cloud_ptr->header;
  calibrated_livox_msg.lidar_id = cloud_msg_ptr->livox_cloud_ptr->lidar_id;
  calibrated_livox_msg.point_num = calibrated_livox_msg.points.size();
  calibrated_livox_msg.timebase = cloud_msg_ptr->livox_cloud_ptr->timebase;
  calibrated_livox_pub.publish(calibrated_livox_msg);

  cloud_array.pop_front();
}

bool flag_exit = false;
void SigHandle(int sig) {
  flag_exit = true;
  ROS_WARN("catch sig %d", sig);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "transform_cloud_node");
  ros::NodeHandle nh("~");

  dev_tools::Logger::Config logger_config;
  logger_config.log_prefix = false;
  std::string package_path = ros::package::getPath("lm_calibr");
  dev_tools::Logger logger(argc, argv, package_path, logger_config);

  // load param
  std::string lidar_topic, encoder_topic;
  nh.param<std::string>("lidar_topic", lidar_topic, std::string(""));
  nh.param<std::string>("encoder_topic", encoder_topic, std::string(""));

  nh.param<double>("time_offset", time_offset, 0.0);

  double d_1, a_1, phi_1, theta_2, d_2, a_2, phi_2;
  nh.param<double>("extrinsic/d_1", d_1, 0.0);
  nh.param<double>("extrinsic/a_1", a_1, 0.0);
  nh.param<double>("extrinsic/phi_1", phi_1, 0.0);
  nh.param<double>("extrinsic/theta_2", theta_2, 0.0);
  nh.param<double>("extrinsic/d_2", d_2, 0.0);
  nh.param<double>("extrinsic/a_2", a_2, 0.0);
  nh.param<double>("extrinsic/phi_2", phi_2, 0.0);

  LOG(INFO) << "[transform_cloud_node] param: " << std::endl
            << "\tlidar_topic: " << lidar_topic << std::endl
            << "\tencoder_topic: " << encoder_topic << std::endl
            << "\ttime_offset: " << time_offset << std::endl
            << "\td_1: " << d_1 << std::endl
            << "\ta_1: " << a_1 << std::endl
            << "\tphi_1: " << phi_1 << std::endl
            << "\ttheta_2: " << theta_2 << std::endl
            << "\td_2: " << d_2 << std::endl
            << "\td_2: " << a_2 << std::endl
            << "\td_2: " << phi_2 << std::endl;

  R_x_phi_1 =
      Eigen::AngleAxisd(phi_1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  R_z_theta_2 =
      Eigen::AngleAxisd(theta_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  R_x_phi_2 =
      Eigen::AngleAxisd(phi_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  t_2 = Eigen::Vector3d(a_1, 0.0, d_1);
  t_1 = Eigen::Vector3d(a_2, 0.0, d_2);

  ros::Subscriber cloub_sub = nh.subscribe(lidar_topic, 1000000, LivoxCallback);
  ros::Subscriber encoder_sub =
      nh.subscribe(encoder_topic, 1000000, EncoderCallback);

  calibrated_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/calibrated_cloud", 10000);
  calibrated_livox_pub =
      nh.advertise<livox_ros_driver::CustomMsg>("/calibrated_livox", 10000);

  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  while (ros::ok()) {
    if (flag_exit) {
      break;
    }

    ros::spinOnce();

    if (SyncMsg()) {
      Process();
    }

    rate.sleep();
  }
}