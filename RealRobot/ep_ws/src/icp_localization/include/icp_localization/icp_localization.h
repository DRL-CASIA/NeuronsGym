#ifndef LIDAR_ODOM_H
#define LIDAR_ODOM_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <stdio.h>
#include <mutex>

#include "icp.h"
#include "localization_config.h"

class ICPLocalization
{
public:
  ICPLocalization()
  {
    LocalizationConfig localization_config;
    localization_config.GetParam(&nh_);
    //! ================== load param ==============================
    odom_frame_ = std::move(localization_config.odom_frame_id);
    imu_odom_frame_ = std::move(localization_config.imu_odom_frame_id);
    base_frame_ = std::move(localization_config.base_frame_id);
    global_frame_ = std::move(localization_config.global_frame_id);
    laser_frame_ = std::move(localization_config.laser_frame_id);
    laser_topic_name_ = std::move(localization_config.laser_topic_name);
    map_topic_name_ = std::move(localization_config.map_topic_name);
    init_pose_topic_name_ = std::move(localization_config.init_pose_topic_name);
    map_file_name_ = std::move(localization_config.map_file_name);
    //! ============================================================
    lidar_init_ = false;
    init_map_transform_ = Eigen::Matrix4d::Identity();
    previous_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    map_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    if(!LoadMapPointCloud(map_file_name_))
    {
      ros::shutdown();
    }
//    if(!GetStaticMap())
//    {
//      ros::shutdown();
//    }
    LoadInitPose(localization_config.initial_pose_x, localization_config.initial_pose_y, localization_config.initial_pose_a);
    pointcloud_sub_ = nh_.subscribe(laser_topic_name_, 1, &ICPLocalization::PointCloudCallback, this);
    init_pose_sub_ = nh_.subscribe(init_pose_topic_name_, 1, &ICPLocalization::InitialPoseCallback,  this);
    pc_pub_  =  nh_.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 10);

  }

  bool GetStaticMap(){
    ros::ServiceClient static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    ros::service::waitForService("static_map", -1);
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;
    if(static_map_srv_.call(req,res)) {

          // ------------------------------------------------------------------------------------------
          cv::Mat grid_map = cv::Mat(res.map.info.height, res.map.info.width,CV_8U,
                                     const_cast<int8_t*>(&res.map.data[0]), (size_t)res.map.info.width);
          cv::Mat detected_edges;
          cv::Canny( grid_map, detected_edges, 100, 200 );
          std::vector<pcl::PointXYZ> edge_points;
          for(int h_i=0; h_i<detected_edges.cols; h_i++){
            for(int w_j=0; w_j<detected_edges.rows; w_j++){
              cv::Scalar pixel = detected_edges.at<uchar>(w_j, h_i);
              if(pixel.val[0]==255){
                pcl::PointXYZ p;
                p.x = res.map.info.origin.position.x + float(h_i)*res.map.info.resolution;
                p.y = res.map.info.origin.position.y + float(w_j)*res.map.info.resolution;
                p.z = 0.05;
                edge_points.push_back(p);
              }
              else {
              }
            }
          }
          pcl::PointCloud<pcl::PointXYZ> map_points;
          map_points.width = edge_points.size();
          map_points.height = 1;
          map_points.is_dense = false;
          map_points.points.resize(map_points.width * map_points.height);
          for(size_t i=0; i<edge_points.size(); i++){
            map_points.points[i] = edge_points[i];
          }
          *map_pointcloud_ = map_points;
          pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::VoxelGrid<pcl::PointXYZ> sor;
          sor.setInputCloud(map_pointcloud_);
          sor.setLeafSize(0.05f, 0.05f, 0.05f);
          sor.filter(*downsample_map);
          *map_pointcloud_ = *downsample_map;
          ROS_INFO("Load pointcloud map successfully!");
          // ------------------------------------------------------------------------------------
          // pcl::io::savePCDFileASCII("/home/drl/ros_codes/qt_ws/src/lidar_ICP/map/map_pc_v2.pcd", *map_pointcloud_);
      return true;
    } else{

      return false;
    }
  }

  bool LoadMapPointCloud(std::string file_name)
  {
    if(pcl::io::loadPCDFile (file_name, *map_pointcloud_) == -1)
    {
      ROS_ERROR("Error reading the map files!\n");
      return false;
    }
    return true;
  }

  void LoadInitPose(double init_x, double init_y, double init_yaw)
  {
    init_map_transform_(0, 0) = cos(init_yaw);
    init_map_transform_(0, 1) = -sin(init_yaw);
    init_map_transform_(1, 0) = sin(init_yaw);
    init_map_transform_(1, 1) = cos(init_yaw);
    init_map_transform_(0, 3) = init_x;
    init_map_transform_(1, 3) = init_y;
    tf::Quaternion quat = tf::createQuaternionFromYaw(init_yaw);
    pose_in_map = tf::Pose(quat, tf::Vector3(init_x, init_y, 0.0));

    InitEKF();
  }

  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &init_pose_msg)
  {
    double init_x, init_y;
    init_x = init_pose_msg->pose.pose.position.x;
    init_y = init_pose_msg->pose.pose.position.y;
    tf::Quaternion quat;
    quat.setX(init_pose_msg->pose.pose.orientation.x);
    quat.setY(init_pose_msg->pose.pose.orientation.y);
    quat.setZ(init_pose_msg->pose.pose.orientation.z);
    quat.setW(init_pose_msg->pose.pose.orientation.w);
    pose_in_map = tf::Pose(quat, tf::Vector3(init_x, init_y, 0.0));
    TFPose2TransformMatrox(pose_in_map, init_map_transform_);
    state_(0) = init_x;
    state_(1) = init_y;
    state_(2) = NormalYaw(tf::getYaw(quat));
  }

  void PublishMapPointCloud()
  {
    sensor_msgs::PointCloud2Ptr pub_seg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_pointcloud_, *pub_seg);
    pub_seg->header.frame_id = "map";
    pub_seg->header.stamp = ros::Time::now();
    pc_pub_.publish(*pub_seg);
  }

  void PublishTFPose(tf::Pose pose, std::string frame_id, std::string child_frame_id, ros::Time time)
  {
    geometry_msgs::TransformStamped lidar_odom_tf_;
    lidar_odom_tf_.header.frame_id = frame_id;
    lidar_odom_tf_.child_frame_id = child_frame_id;
    lidar_odom_tf_.header.stamp = time + ros::Duration(0.1);
    lidar_odom_tf_.transform.translation.x = pose.getOrigin().x();
    lidar_odom_tf_.transform.translation.y = pose.getOrigin().y();
    lidar_odom_tf_.transform.translation.z = pose.getOrigin().z();
    lidar_odom_tf_.transform.rotation.x = pose.getRotation().x();
    lidar_odom_tf_.transform.rotation.y = pose.getRotation().y();
    lidar_odom_tf_.transform.rotation.z = pose.getRotation().z();
    lidar_odom_tf_.transform.rotation.w = pose.getRotation().w();
    tf_broadcaster_.sendTransform(lidar_odom_tf_);
  }

  void TransforMaxtrix2TFPose(Eigen::Matrix4d transform, tf::Pose &pose)
  {
    tf::Matrix3x3 tf_mat;
    Eigen::Matrix3d rotation_mat = Eigen::Matrix3d::Identity();
    rotation_mat(0, 0) = transform(0, 0);
    rotation_mat(0, 1) = transform(0, 1);
    rotation_mat(0, 2) = transform(0, 2);
    rotation_mat(1, 0) = transform(1, 0);
    rotation_mat(1, 1) = transform(1, 1);
    rotation_mat(1, 2) = transform(1, 2);
    rotation_mat(2, 0) = transform(2, 0);
    rotation_mat(2, 1) = transform(2, 1);
    rotation_mat(2, 2) = transform(2, 2);
    tf::matrixEigenToTF(rotation_mat, tf_mat);

    tf::Quaternion quat;
    tf_mat.getRotation(quat);
    pose = tf::Pose(quat, tf::Vector3(transform(0,3), transform(1, 3), 0.0));
  }

  void TFPose2TransformMatrox(tf::Pose pose, Eigen::Matrix4d &transform)
  {
    transform = Eigen::Matrix4d::Identity();
    double yaw = tf::getYaw(pose.getRotation());
    transform(0, 0) = cos(yaw);
    transform(0, 1) = -sin(yaw);
    transform(1, 0) = sin(yaw);
    transform(1, 1) = cos(yaw);
    transform(0, 3) = pose.getOrigin().x();
    transform(1, 3) = pose.getOrigin().y();
  }

  void InitEKF()
  {
    P_ = Eigen::Matrix3d::Zero();

    K_ = Eigen::Matrix3d::Identity();

    state_ = Eigen::Vector3d(pose_in_map.getOrigin().x(), pose_in_map.getOrigin().y(),
                             tf::getYaw(pose_in_map.getRotation()));
  }

  double GetQFactor(double x)
  {
    double low = 0.01;
    double up = 0.04;
    double Q_min = 0.00001;
    double Q_max = 0.001;
    if(x<low)
    {
      return Q_min;
    }
    else if (x>up) {
      return Q_max;
    }
    else {
      return ((x-low)/(up-low)) *(Q_max-Q_min) + Q_min;
    }
  }

  double GetRFactor(int x)
  {
    int low = 100;
    int up = 200;
    double R_min = 0.001;
    double R_max = 0.01;
    if(x<low)
    {
      return R_max;
    }
    else if (x>up) {
      return R_min;
    }
    else
    {
      return (1.0 - ((double) (x-low))/((double)(up-low))) * (R_max-R_min) + R_min;
    }
  }

  double NormalYaw(double yaw)
  {
    double pi = 3.1415926;
    if(yaw > pi)
    {
      return (yaw - 2*pi);
    }
    if(yaw < -pi)
    {
      return (yaw + 2*pi);
    }
    return yaw;
  }

  Eigen::Matrix3d GetFMatrix(Eigen::Vector3d state, tf::Pose imu_relative_pose)
  {
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    double imu_x = imu_relative_pose.getOrigin().x();
    double imu_y = imu_relative_pose.getOrigin().y();
    F(0, 2) = -imu_x * std::sin(state(2)) - imu_y * std::cos(state(2));
    F(1, 2) = imu_x * std::cos(state(2)) - imu_y * std::sin(state(2));
    return F;
  }

  void EKFPredict(tf::Pose imu_relative_pose, tf::Pose odom_relative_pose)
  {
    double imu_x = imu_relative_pose.getOrigin().x();
    double imu_y = imu_relative_pose.getOrigin().y();
    double imu_yaw = tf::getYaw(imu_relative_pose.getRotation());
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    double delta_x = std::fabs(imu_x-odom_relative_pose.getOrigin().x());
    double delta_y = std::fabs(imu_y-odom_relative_pose.getOrigin().y());
    double delta_yaw = imu_yaw - tf::getYaw(odom_relative_pose.getRotation());
    delta_yaw = NormalYaw(delta_yaw);
    double Q_x = GetQFactor(delta_x);
    double Q_y = GetQFactor(delta_y);
    double Q_yaw = GetQFactor(delta_yaw);
    ROS_INFO("Q_x: %f, Q_y: %f, Q_yaw: %f", Q_x, Q_y, Q_yaw);
    Q(0,0) = Q_x;
    Q(1,1) = Q_y;
    Q(2,2) = Q_yaw;
    Eigen::Vector3d estimated_state;
    estimated_state(0) = state_(0) + imu_x * std::cos(state_(2)) - imu_y * std::sin(state_(2));
    estimated_state(1) = state_(1) + imu_x * std::sin(state_(2)) + imu_y * std::cos(state_(2));
    estimated_state(2) = state_(2) + imu_yaw;

    Eigen::Matrix3d F = GetFMatrix(state_, imu_relative_pose);
    P_ = F * P_ * F.transpose() + Q;

    state_ = estimated_state;
    state_(2) = NormalYaw(state_(2));
  }

  void EKFUpdate(tf::Pose pose_in_map, int num_correspondences)
  {
    Eigen::Vector3d z, y;
    z(0) = pose_in_map.getOrigin().x();
    z(1) = pose_in_map.getOrigin().y();
    z(2) = NormalYaw(tf::getYaw(pose_in_map.getRotation()));
    y = z - state_;
    ROS_INFO("EKF error: %f", y(2));
    y(2) = NormalYaw(y(2));
    ROS_INFO("EKF error: %f, %f, %f", y(0), y(1), y(2));
    double R_i = GetRFactor(num_correspondences);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0,0) = R_i;
    R(1,1) = R_i;
    R(2,2) = R_i;
    ROS_INFO("R: %f", R_i);
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    K_ = P_ * H.transpose() * S.inverse();
    state_ = state_ + K_ * y;
    P_ = (I - K_ * H ) * P_;
  }

  void State2TFPose(Eigen::Vector3d state, tf::Pose &pose)
  {
    tf::Quaternion quat = tf::createQuaternionFromYaw(state(2));
    pose = tf::Pose(quat, tf::Vector3(state(0), state(1), 0.0));
  }


  bool GetTransform(std::string frame_id, std::string child_frame_id,
                    ros::Time time, tf::StampedTransform &transform)
  {
    try {
      tf_listener_.lookupTransform(frame_id, child_frame_id, time, transform);
      return true;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("[lidar_odom_node:->GetTransform] Cannot lookup transform between %s and %s",
                 frame_id.c_str(), child_frame_id.c_str());
      return false;
    }
  }

  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pCloud)
  {
    ros::Time startTime;
    startTime = pCloud->header.stamp;
    pcl::PointCloud<pcl::PointXYZ> in_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);;
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*pCloud, in_cloud);
    *cloud = in_cloud;
    // ROS_INFO("cloud size: %d", cloud->size());
    // ROS_INFO("cloud point size: %d", cloud->points.size());
    ROS_INFO("Received pointcolud");

    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud);



    if(!lidar_init_)
    {
      if(GetTransform(odom_frame_, laser_frame_, startTime, odom_transform_))
      {
        lidar_init_ = true;
//        start_time = startTime.toSec();

      }
      else {
        ROS_INFO("Waiting odom to base_laser_link transform");
        return;
      }
      // if(GetTransform(odom_frame_, imu_odom_frame_, startTime, imu_transform_))
      // {
      //   lidar_init_ = true;
      // }
      // else {
      //   ROS_INFO("Waiting odom to imu_odom_link transform");
      //   lidar_init_ = false;
      // }

    }
    else {

      ROS_INFO("----------------------------------------------------------------");
      ROS_INFO("debug count: %d", debug_count);

      tf::StampedTransform current_imu_transform;
      tf::StampedTransform current_odom_transform;
      Eigen::Matrix4d odom_predict_transform = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d odom_transform = Eigen::Matrix4d::Identity();

      // tf::Pose relative_imu_pose;
      // double delta_imu_x, delta_imu_y, delta_imu_z, delta_imu_yaw;
      // if(GetTransform(odom_frame_, imu_odom_frame_, startTime, current_imu_transform))
      // {
      //   tf::Pose current_imu_pose = tf::Pose(current_imu_transform.getRotation(), current_imu_transform.getOrigin());
      //   tf::Pose previous_imu_pose = tf::Pose(imu_transform_.getRotation(), imu_transform_.getOrigin());
      //   relative_imu_pose = previous_imu_pose.inverse() * current_imu_pose;
        
      //   delta_imu_x = relative_imu_pose.getOrigin().x();
      //   delta_imu_y = relative_imu_pose.getOrigin().y();
      //   delta_imu_yaw = tf::getYaw(relative_imu_pose.getRotation());
      //   ROS_INFO("delta imu_odom: %f, %f, %f", delta_imu_x, delta_imu_y, delta_imu_yaw);

      // }
      tf::Pose relative_odom_pose;
      tf::Pose current_odom_pose;
      if(GetTransform(odom_frame_, laser_frame_, startTime, current_odom_transform))
      {
        current_odom_pose = tf::Pose(current_odom_transform.getRotation(), current_odom_transform.getOrigin());
        tf::Pose previous_pose = tf::Pose(odom_transform_.getRotation(), odom_transform_.getOrigin());
        relative_odom_pose = previous_pose.inverse() * current_odom_pose;
        double x, y, z, yaw;
        x = relative_odom_pose.getOrigin().x();
        y = relative_odom_pose.getOrigin().y();
        yaw = tf::getYaw(relative_odom_pose.getRotation());
        ROS_INFO("delta odom: %f, %f, %f", x, y, yaw);
//        log_file<<x<<" "<<y<<" "<<yaw<<" ";
        odom_transform(0, 0) = cos(yaw);
        odom_transform(0, 1) = -sin(yaw);
        odom_transform(1, 0) = sin(yaw);
        odom_transform(1, 1) = cos(yaw);
        odom_transform(0, 3) = x;
        odom_transform(1, 3) = y;

        odom_predict_transform = init_map_transform_ * odom_transform;

        pcl::transformPointCloud(*cloud, *in_map_cloud, odom_predict_transform.cast<float>());
      }
      else {
        return;
      }
      

      EKFPredict(relative_odom_pose, relative_odom_pose);

      if(cloud->points.size()<40)
      {
        ROS_INFO("point number: %d", (int)cloud->points.size());
        // tf::Pose ekf_pose;
        // State2TFPose(state_, ekf_pose);
        // pose_in_map = ekf_pose;
        // TFPose2TransformMatrox(pose_in_map, init_map_transform_);
        // TransforMaxtrix2TFPose(init_map_transform_, pose_in_map);
        TransforMaxtrix2TFPose(odom_predict_transform, pose_in_map);

        tf::Pose odom_in_map = pose_in_map * current_odom_pose.inverse();
        PublishTFPose(odom_in_map, global_frame_, odom_frame_, startTime);
        last_odom_in_map_ = odom_in_map;
        // there are some problem
        return;
      }


      Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity();

      tf::Pose init_lidar_pose;
      TransforMaxtrix2TFPose(odom_predict_transform, init_lidar_pose);
      ROS_INFO("init localization: %f, %f, %f", init_lidar_pose.getOrigin().x(), init_lidar_pose.getOrigin().y(),
                                              tf::getYaw(init_lidar_pose.getRotation()));
      std::chrono::high_resolution_clock::time_point time_begin = std::chrono::high_resolution_clock::now();
      int num_correspondences=100;
      naive_icp(in_map_cloud, map_pointcloud_, init_transform, num_correspondences);
      ROS_INFO("ICP time:%lf", std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_begin).count());

//      pcl::transformPointCloud(*cloud, *in_map_cloud, (init_transform*odom_predict_transform).cast<float>());
//      *add_cloud = *map_pointcloud_ + *in_map_cloud;
//      char file_name[100];
//      std::sprintf(file_name, "/home/drl/ros_codes/qt_ws/src/lidar_ICP/match_data/debug_cloud_%d.pcd", debug_count);
//      pcl::io::savePCDFileASCII(file_name, *add_cloud);
      tf::Pose lidar_relative_pose;
      ROS_INFO("num_correspondences: %d", num_correspondences);
      TransforMaxtrix2TFPose(init_transform, lidar_relative_pose);
      ROS_INFO("lidar collection: %f, %f, %f", lidar_relative_pose.getOrigin().x(), lidar_relative_pose.getOrigin().y(),
                                              tf::getYaw(lidar_relative_pose.getRotation()));

      tf::Pose lidar_delta_pose;
      TransforMaxtrix2TFPose(init_transform*odom_transform, lidar_delta_pose);
      ROS_INFO("delta lidar_icp: %f, %f, %f", lidar_delta_pose.getOrigin().x(), lidar_delta_pose.getOrigin().y(),
                                              tf::getYaw(lidar_delta_pose.getRotation()));


//      log_file<<lidar_delta_pose.getOrigin().x()<<" "<<lidar_delta_pose.getOrigin().y()<<" "<<tf::getYaw(lidar_delta_pose.getRotation())<<" ";
      if(std::fabs(lidar_relative_pose.getOrigin().x())>0.6 ||
         std::fabs(lidar_relative_pose.getOrigin().y()>0.6) ||
         std::fabs(tf::getYaw(lidar_relative_pose.getRotation()))>0.3)
      {
        init_map_transform_ = odom_predict_transform;
      }
      else
      {
        init_map_transform_ = init_transform * odom_predict_transform;
      }
//      init_map_transform_ = init_transform * odom_predict_transform;
      tf::Pose lidar_localize_pose;
      TransforMaxtrix2TFPose(init_map_transform_, lidar_localize_pose);
      EKFUpdate(lidar_localize_pose, num_correspondences);
      tf::Pose ekf_pose;
      State2TFPose(state_, ekf_pose);
      tf::Pose ekf_relative_pose = pose_in_map.inverse() * ekf_pose;
      ROS_INFO("delta ekf: %f, %f, %f", ekf_relative_pose.getOrigin().x(), ekf_relative_pose.getOrigin().y(),
                                              tf::getYaw(ekf_relative_pose.getRotation()));
//      log_file<<ekf_relative_pose.getOrigin().x()<<" "<<ekf_relative_pose.getOrigin().y()<<" "<<tf::getYaw(ekf_relative_pose.getRotation())<<"\n";
      pose_in_map = ekf_pose;
    //  TransforMaxtrix2TFPose(init_map_transform_, pose_in_map);
      TFPose2TransformMatrox(pose_in_map, init_map_transform_);

      tf::Pose odom_in_map = pose_in_map * current_odom_pose.inverse();
      PublishTFPose(odom_in_map, global_frame_, odom_frame_, startTime);

      last_odom_in_map_ = odom_in_map;

      odom_transform_ = current_odom_transform;
      imu_transform_ = current_imu_transform;
    }

    PublishMapPointCloud();

    debug_count += 1;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_, init_pose_sub_;
  ros::Publisher odom_pub_, pc_pub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pointcloud_, map_pointcloud_;
  Eigen::Matrix4d init_map_transform_;
  bool lidar_init_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform odom_transform_, imu_transform_;
//  nav_msgs::Odometry imu_odom_, previous_imu_odom_, current_imu_odom_;
//  std::ofstream log_file;
  double start_time;

  tf::Pose pose_in_map;

  // debug
  int debug_count=0;
  // ekf paramter
  Eigen::Matrix3d P_, K_;
  Eigen::Vector3d state_;

  // config
  std::string odom_frame_, imu_odom_frame_, base_frame_, global_frame_, laser_frame_;
  std::string laser_topic_name_, map_topic_name_, init_pose_topic_name_;
  std::string map_file_name_;

  tf::Pose last_odom_in_map_;

};



#endif // LIDAR_ODOM_H
