#ifndef LOCALIZATION_CONFIG_H
#define LOCALIZATION_CONFIG_H
#include <ros/ros.h>
// Get Parameters from ROS parameter server
struct LocalizationConfig {
  void GetParam(ros::NodeHandle *nh) {
      nh->param<std::string>("odom_frame_id", odom_frame_id, "odom");
      nh->param<std::string>("base_frame_id", base_frame_id, "base_link");
      nh->param<std::string>("global_frame_id", global_frame_id, "map");
      nh->param<std::string>("imu_odom_frame_id", imu_odom_frame_id, "imu_odom_link");
      nh->param<std::string>("laser_frame_id", laser_frame_id, "laser_link");
      nh->param<std::string>("laser_topic_name", laser_topic_name, "/corrected_pointcloud");
      nh->param<std::string>("map_topic_name", map_topic_name, "map");
      nh->param<std::string>("init_pose_topic_name", init_pose_topic_name, "initialpose");
      nh->param<std::string>("map_file_name", map_file_name, "/home/drl/ros_codes/qt_ws/src/lidar_ICP/map/map_pc_v2.pcd");

      nh->param<double>("initial_pose_x", initial_pose_x, 0.5);
      nh->param<double>("initial_pose_y", initial_pose_y, 0.5);
      nh->param<double>("initial_pose_a", initial_pose_a, 1.57);

  }
  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;
  std::string imu_odom_frame_id;
  std::string laser_frame_id;

  std::string laser_topic_name;
  std::string map_topic_name;
  std::string init_pose_topic_name;

  std::string map_file_name;


  double initial_pose_x;
  double initial_pose_y;
  double initial_pose_a;

};

#endif // LOCALIZATION_CONFIG_H
