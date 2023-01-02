#include "icp_localization/imu_odom.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "imu_preint");

  IMUPreintegration ImuP;

  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

  ros::spin();

  return 0;
}
