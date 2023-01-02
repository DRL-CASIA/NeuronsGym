#ifndef IMU_ODOM_H
#define IMU_ODOM_H

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;
template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


class IMUPreintegration
{
public:
  ros::NodeHandle nh;

  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  // odom -> base_link
  tf::TransformBroadcaster tfOdom2BaseLink;

  double imu_x = 0.0;
  double imu_y = 0.0;
  double imu_yaw = 0.0;
  double imu_vx = 0.0;
  double imu_vy = 0.0;
  double imu_ax = 0.0;
  double imu_ay = 0.0;
  double imu_vw = 0.0;
  bool imu_init = false;
  double last_imu_time_;

  double frame_num = 0.0;
  double frame_ax_sum = 0.0;
  double frame_ay_sum = 0.0;
  double frame_vw_sum = 0.0;
  double bias_ax = 0.0;
  double bias_ay = 0.0;
  double bias_vw = 0.0;

  bool yaw_init_ = false;
  double odom_yaw_ = 0.0;

  std::ofstream log_file;
  IMUPreintegration()
  {
    log_file.open("/home/drl/ros_codes/qt_ws/src/lidar_ICP/scipts/imu_odom_data_1.txt");
    subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 5,    &IMUPreintegration::odomCollector, this, ros::TransportHints().tcpNoDelay());
    subImu      = nh.subscribe<sensor_msgs::Imu>  ("/imu", 2000, &IMUPreintegration::imuPreintagration, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry> ("/imu_odom", 1);
  }

  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
  {
      sensor_msgs::Imu imu_out = imu_in;
      // rotate acceleration

      imu_ax = imu_in.linear_acceleration.x;
      imu_ay = imu_in.linear_acceleration.y;
      imu_vw = imu_in.angular_velocity.z;

      imu_out.linear_acceleration.x = imu_in.linear_acceleration.y - bias_ay; // + 0.012236381790878577+8.91640724e-04+0.00061005;
      imu_out.linear_acceleration.y = imu_in.linear_acceleration.x - bias_ax; //- 0.07224487771046464-2.24728523e-03+0.00050388;
      imu_out.linear_acceleration.z = 0.0;
      // rotate gyroscope

      double vw = imu_in.angular_velocity.z - bias_vw;//- 0.02119440275306317-7.34807279e-05;
      vw = -vw * 3.1415926 / 180.0;
      imu_out.angular_velocity.x = 0.0;
      imu_out.angular_velocity.y = 0.0;
      imu_out.angular_velocity.z = vw;


      imu_out.orientation.x = imu_in.orientation.x;
      imu_out.orientation.y = imu_in.orientation.y;
      imu_out.orientation.z = -imu_in.orientation.z;
      imu_out.orientation.w = imu_in.orientation.w;

      return imu_out;
  }

  void odomCollector(const nav_msgs::Odometry::ConstPtr& odomMsg)
  {
    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;

    float v_x = odomMsg->twist.twist.linear.x;
    float v_y = odomMsg->twist.twist.linear.y;
    float v_w = odomMsg->twist.twist.angular.z;
    if(!yaw_init_)
    {
      tf::Pose init_pose;
      tf::poseMsgToTF(odomMsg->pose.pose, init_pose);
      odom_yaw_ = tf::getYaw(init_pose.getRotation());
      ROS_INFO("odom yaw: %f", odom_yaw_);
      yaw_init_ = true;
    }

    double base_imu_vx = v_x * cos(imu_yaw) - v_y * sin(imu_yaw);
    double base_imu_vy = v_x * sin(imu_yaw) + v_y * cos(imu_yaw);

    if (((v_x*v_x+v_y*v_y)<0.0001) && (v_w < 0.001))
    {
      imu_vx = 0.0;
      imu_vy = 0.0;
      imu_vw = 0.0;

      bias_ax = bias_ax * frame_num / (frame_num + 1.0) + imu_ax / (frame_num + 1.0);
      bias_ay = bias_ay * frame_num / (frame_num + 1.0) + imu_ay / (frame_num + 1.0);
      bias_vw = bias_vw * frame_num / (frame_num + 1.0) + imu_vw / (frame_num + 1.0); ;
      frame_num += 1.0;
    }
    else {

      if(std::fabs(base_imu_vx) < std::fabs(imu_vx))
      {
        imu_vx = base_imu_vx;
      }
      if(std::fabs(base_imu_vy) < std::fabs(imu_vy))
      {
        imu_vy = base_imu_vy;
      }
//      imu_vx = std::min(base_imu_vx, imu_vx);
//      imu_vy = std::min(base_imu_vy, imu_vy);
//      if(std::fabs(v_x-base_imu_vx)>0.05 || std::fabs(v_y-base_imu_vy) > 0.05)
//      {

//      }
//      else {
//        imu_vx = v_x * cos(imu_yaw) - v_y * sin(imu_yaw);
//        imu_vy = v_x * sin(imu_yaw) + v_y * cos(imu_yaw);
//      }
    }

    log_file<<imu_vx<<" "<<imu_vy<<" "<<imu_vw<<" "<<base_imu_vx<<" "<<base_imu_vy<<" "<<v_w<<"\n";
  }

  void imuPreintagration(const sensor_msgs::Imu::ConstPtr& imu_raw)
  {
    if(!yaw_init_)
    {
      return;
    }
    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
    if(!imu_init)
    {
      last_imu_time_ = ROS_TIME(imu_raw);
      imu_yaw = odom_yaw_;
      ROS_INFO("imu init yaw: %f", imu_yaw);
      imu_init = true;
    }
    else {
      double ax = thisImu.linear_acceleration.x * cos(imu_yaw) - thisImu.linear_acceleration.y * sin(imu_yaw);
      double ay = thisImu.linear_acceleration.x * sin(imu_yaw) + thisImu.linear_acceleration.y * cos(imu_yaw);
      double vw = thisImu.angular_velocity.z;
      double delta_t = (ROS_TIME(imu_raw) - last_imu_time_);

      imu_vx = imu_vx + ax * delta_t;
      imu_vy = imu_vy + ay * delta_t;

      imu_x = imu_x + imu_vx * delta_t + 0.0 * ax * delta_t * delta_t;
      imu_y = imu_y + imu_vy * delta_t + 0.0 * ay * delta_t * delta_t;
      imu_yaw = imu_yaw + vw * delta_t;
      nav_msgs::Odometry odometry;
      odometry.header.stamp = thisImu.header.stamp;
      odometry.header.frame_id = "imu_odom";
      odometry.pose.pose.position.x = imu_x;
      odometry.pose.pose.position.y = imu_y;
      odometry.pose.pose.position.z = 0.0;

      tf::Quaternion quat = tf::createQuaternionFromYaw(imu_yaw);

      odometry.pose.pose.orientation.x = quat.x();
      odometry.pose.pose.orientation.y = quat.y();
      odometry.pose.pose.orientation.z = quat.z();
      odometry.pose.pose.orientation.w = quat.w();

      odometry.twist.twist.linear.x = imu_vx * cos(imu_yaw) + imu_vy * sin(imu_yaw);
      odometry.twist.twist.linear.y = -imu_vx * sin(imu_yaw) + imu_vy * cos(imu_yaw);
      odometry.twist.twist.linear.z = 0.0;
      odometry.twist.twist.angular.x = 0.0;
      odometry.twist.twist.angular.y = 0.0;
      odometry.twist.twist.angular.z = vw;
      pubImuOdometry.publish(odometry);
      last_imu_time_ = ROS_TIME(imu_raw);

      // publish transformation
      tf::Transform tCur;
      tf::poseMsgToTF(odometry.pose.pose, tCur);
      tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, thisImu.header.stamp, "odom", "imu_odom_link");
      tfOdom2BaseLink.sendTransform(odom_2_baselink);
    }

  }


};


#endif // IMU_ODOM_H
