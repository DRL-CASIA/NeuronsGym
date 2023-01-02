#include "icp_localization/icp_localization.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "icp_localization");

  ICPLocalization LidarLocalization;

  ROS_INFO("\033[1;32m----> Lidar ICP Localization Started.\033[0m");

  ros::spin();

  return 0;
}
