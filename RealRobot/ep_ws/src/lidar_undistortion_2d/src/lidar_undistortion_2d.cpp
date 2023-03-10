#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

class LidarMotionCalibrator
{
private:
    
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_, depth_scan_sub_;
    ros::Publisher scan_pub_;
    ros::Publisher pointcloud_pub_;

    tf::TransformListener* tf_;

    std::string scan_sub_topic_;
    std::string scan_pub_topic_;
    std::string pointcloud_pub_topic_;
    std::string lidar_frame_;
    std::string odom_frame_;



    bool enable_pub_pointcloud_, received_depth_scan_;
    double lidar_scan_time_gain_;

    sensor_msgs::LaserScan depth_camera_scan_;

public:

    LidarMotionCalibrator(ros::NodeHandle node_handle)
    {
        nh_ = node_handle;
        tf_ = new tf::TransformListener(ros::Duration(10.0));

        received_depth_scan_ = false;

        ros::NodeHandle nh_param("~");
        nh_param.param<std::string>("scan_sub_topic", scan_sub_topic_, "/scan");
        nh_param.param<std::string>("scan_pub_topic", scan_pub_topic_, "/undistortion_scan");
        nh_param.param<bool>("enable_pub_pointcloud", enable_pub_pointcloud_, true);
        nh_param.param<std::string>("point_cloud_pub_topic", pointcloud_pub_topic_, "/undistortion_pointcloud");
        nh_param.param<std::string>("lidar_frame", lidar_frame_, "base_laser_link");
        nh_param.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_param.param<double>("lidar_scan_time_gain", lidar_scan_time_gain_, 1.0);

        scan_sub_ = nh_.subscribe(scan_sub_topic_, 10, &LidarMotionCalibrator::ScanCallBack, this);
        depth_scan_sub_ = nh_.subscribe("/depth_camera_scan", 10, &LidarMotionCalibrator::DepthCameraScanCallBack, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_pub_topic_, 1);
        if(enable_pub_pointcloud_) pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_pub_topic_, 1);
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL) delete tf_;
    }

    void DepthCameraScanCallBack(const sensor_msgs::LaserScanConstPtr& depth_scan_msg)
    {
      depth_camera_scan_ = *depth_scan_msg;
      if(!received_depth_scan_)
      {
        received_depth_scan_ = true;
      
      }
     // ROS_INFO("received depth camera scan msg");
    }

    // ??????????????????????????????????????????
    void ScanCallBack(const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
        // ??????????????????????????????
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        // ????????????????????????
        int beamNum = scan_msg->ranges.size();
        endTime = startTime + ros::Duration(scan_msg->time_increment * lidar_scan_time_gain_ * beamNum);

        // ?????????????????????
        std::vector<double> angles;
        std::vector<float> ranges;
        std::vector<float> intensities;

        for(int alpha = beamNum - 1; alpha >= 0; alpha--)
        {
            int inverse_alpha = beamNum - 1 - alpha;
            double lidar_dist = scan_msg->ranges[inverse_alpha];

            if(std::isnan(lidar_dist) || lidar_dist < scan_msg->range_min)
                lidar_dist = 0.0;

            intensities.push_back(scan_msg->intensities[inverse_alpha]);
            ranges.push_back(lidar_dist);
            angles.push_back(scan_msg->angle_min + scan_msg->angle_increment * inverse_alpha);
        }

        tf::Stamped<tf::Pose> start_pose, end_pose;
        if(!getLaserPose(start_pose, ros::Time(startTime), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }
        if(!getLaserPose(end_pose,ros::Time(endTime), tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        // ROS_INFO("calibration start");
        // ????????????
        Lidar_Calibration(ranges,
                          angles,
                          startTime,
                          endTime,
                          tf_);

        // ROS_INFO("calibration end");

        if(enable_pub_pointcloud_) publishPointCloud2(startTime, angles, ranges, intensities);  
        publishScan(scan_msg, ranges, angles, intensities, start_pose, end_pose);
    }

    

    void publishPointCloud2(ros::Time start_time,
                            std::vector<double>& angles,
                            std::vector<float>& ranges,
                            std::vector<float>& intensities)
    {
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
        pcl::PointXYZ point_xyzi;

        for(int index = 0; index < angles.size(); ++index) {
            if(ranges[index] < 0.3)
            {
                continue;
            }
            if(std::isinf(ranges[index]) || std::isnan(ranges[index]))
            {
                continue;
            }
            point_xyzi.x = ranges[index] * cos(angles[index]);
            point_xyzi.y = ranges[index] * sin(angles[index]);
            point_xyzi.z = 0.0;
            
            // point_xyzi.intensity = intensities[index];
            pointcloud_pcl.points.push_back(point_xyzi);
        }

        pcl::toROSMsg(pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = lidar_frame_;
        pointcloud_msg.header.stamp = start_time;
        pointcloud_pub_.publish(pointcloud_msg);
    }


    /**
     * @name getLaserPose()
     * @brief ????????????????????????????????????????????????tf::Pose
     *        ??????dt?????????????????????odom??????????????????
     * @param odom_pos  ??????????????????
     * @param dt        dt??????
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = lidar_frame_;
        robot_pose.stamp_ = dt;   //?????????ros::Time()?????????????????????????????????

        // get the global pose of the robot
        tf::StampedTransform transform_laser;
        try
        {
//          tf_->lookupTransform(odom_frame_, lidar_frame_, ros::Time(0), transform_laser);
            if(!tf_->waitForTransform(odom_frame_, lidar_frame_, dt, ros::Duration(0.5)))             // 0.15s ?????????????????????
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose(odom_frame_, robot_pose, odom_pose);
//          odom_pose.setData(transform_laser);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        ??????????????????????????????????????????;
     *        ?????????????????????????????????????????????????????????
     * @param frame_base_pose       ????????????????????????????????????
     * @param frame_start_pose      ??????????????????????????????????????????
     * @param frame_end_pose        ?????????????????????????????????????????????
     * @param ranges                ????????????????????????
     * @param angles                ????????????????????????
     * @param startIndex            ???????????????????????????????????????????????????
     * @param beam_number           ???????????????????????????
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<float>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
        //TODO
        // ??????????????????????????????????????????
        const double beam_step = 1.0 / (beam_number - 1);

        // ???????????????
        tf::Quaternion base_quaternion = frame_base_pose.getRotation();
        double base_angle = tf::getYaw(base_quaternion);
        tf::Quaternion start_quaternion = frame_start_pose.getRotation();
        double start_angle = tf::getYaw(start_quaternion);
        tf::Quaternion end_quaternion = frame_end_pose.getRotation();
        double end_angle = tf::getYaw(end_quaternion);

//        ROS_INFO("base angle: %f, start angle: %f, end angle: %f", base_angle, start_angle, end_angle);

        // ????????????
        tf::Vector3 base_point = frame_base_pose.getOrigin();
        base_point.setZ(0);
        tf::Vector3 start_point = frame_start_pose.getOrigin();
        start_point.setZ(0);
        tf::Vector3 end_point = frame_end_pose.getOrigin();
        end_point.setZ(0);

        for(int index = 0; index < beam_number; ++index) {

            const double lidar_range = ranges[startIndex + index];
            const double lidar_angle = angles[startIndex + index];

            // ????????????
            tf::Pose frame_cur_pose;
            tf::Quaternion cur_quaternion = start_quaternion.slerp(end_quaternion, index * beam_step);
            tf::Vector3 cur_point = start_point.lerp(end_point, index * beam_step);
            frame_cur_pose.setOrigin(cur_point);
            frame_cur_pose.setRotation(cur_quaternion);

            // ??????????????????????????????????????????0
            if(!tfFuzzyZero(lidar_range) && !std::isinf(lidar_range)) {

                tf::Vector3 lidar_point;
                double lidar_x = lidar_range * cos(lidar_angle);
                double lidar_y = lidar_range * sin(lidar_angle);
                lidar_point.setValue(lidar_x, lidar_y, 0);
                
                tf::Vector3 corrected_lidar_point = tf::Pose(base_quaternion, base_point).inverse() * frame_cur_pose * lidar_point;

                ranges[startIndex + index] = hypot(corrected_lidar_point.getX(), corrected_lidar_point.getY());
                angles[startIndex + index] = atan2(corrected_lidar_point.getY(), corrected_lidar_point.getX());
            }
            else { // ?????????0

                // ???????????????????????????
                double tmp_angle = tf::getYaw(cur_quaternion) + lidar_angle;
                tmp_angle = tfNormalizeAngle(tmp_angle);

                // ?????????????????? ?????????????????????????????????????????????????????????start_pos?????????????????????
                angles[startIndex + index] = tfNormalizeAngle(tmp_angle - base_angle);
            }

            // ROS_INFO("\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", 
            // startIndex + index, base_point.getX(), base_point.getY(), tf::getYaw(base_quaternion), 
            //                     cur_point.getX(), cur_point.getY(), tf::getYaw(cur_quaternion), 
            // lidar_range, lidar_angle, ranges[startIndex + index], angles[startIndex + index]);
        }
        //end of TODO
    }



    //??????????????????????????????????????????????????????????????????10ms
    //???????????????Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief ??????????????????????????????????????????????????????????????????5ms
     * @param ranges ???????????????????????????
     * @param angle??????????????????????????????
     * @param startTime??????????????????????????????
     * @param endTime?????????????????????????????????
     * @param tf_
    */
    void Lidar_Calibration(std::vector<float>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //????????????????????????
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5ms???????????????
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //???????????? us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; // ?????????????????????????????????

        //?????????????????????????????????
        int start_index = 0;

        //?????????????????? ?????????????????????????????????????????????????????????????????????base_pose
        //??????????????????????????????????????????????????????base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0), tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        //??????????????????????????????????????????
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //????????????,?????????????????????interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(i == beamNumber - 1) {
                nh_.param<std::string>("odom_frame", odom_frame_, "odom");
            }
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //??????????????????????????????
                //???????????????
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //???????????????????????????????????????
                //interpolation_time_duration?????????????????????.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose, // ??????????????????
                                        frame_start_pose, // ????????????????????????????????????
                                        frame_mid_pose, // ????????????????????????????????????
                                        ranges, // ????????????????????????
                                        angles, // ????????????????????????
                                        start_index, // ??????????????????????????????index
                                        interp_count); // ????????????????????????

                //????????????
                start_time = mid_time;
                start_index = i + 1; // ???????????????????????????BUG
                frame_start_pose = frame_mid_pose;
            }
        }
    }

    void publishScan(const sensor_msgs::LaserScanConstPtr& scan_msg,
                     std::vector<float>& ranges,
                     std::vector<double>& angles,
                     std::vector<float>& intensities,
                     tf::Stamped<tf::Pose>& start_pose,
                     tf::Stamped<tf::Pose>& end_pose)
    {
        
        sensor_msgs::LaserScan publish_msg;
        publish_msg.header = scan_msg->header;
        publish_msg.time_increment = scan_msg->time_increment;
        publish_msg.scan_time = scan_msg->scan_time;
        publish_msg.range_min = scan_msg->range_min;
        publish_msg.range_max = scan_msg->range_max;

        publish_msg.angle_max = scan_msg->angle_max;
        publish_msg.angle_min = scan_msg->angle_min + M_PI; // why need add 3.14?
        publish_msg.angle_increment = scan_msg->angle_increment;


        for(int alpha = 0; alpha < ranges.size(); ++alpha) {
            publish_msg.ranges.push_back(0);
            publish_msg.intensities.push_back(0);
        }

        if(received_depth_scan_)
        {
          for (size_t i=0; i< depth_camera_scan_.ranges.size(); i++) {
            double angle = depth_camera_scan_.angle_increment * double(i) + depth_camera_scan_.angle_min;
            if(angle < 0)
            {
              angle += 2 * M_PI;
            }
            int index = (int)((angle - publish_msg.angle_min) / publish_msg.angle_increment);
            if(index >= 0 && index < ranges.size())
            {
		//    std::printf("insert depth image scan");
              publish_msg.ranges[index] = depth_camera_scan_.ranges[i];
              publish_msg.intensities[index] = 100;
            }
          }
        }

        for(int alpha = ranges.size() - 1; alpha >= 0; --alpha) {
            double angle = (angles[alpha] < 0 || tfFuzzyZero(angles[alpha])) ? angles[alpha] + 2 * M_PI : angles[alpha];
            angle += publish_msg.angle_min;
            int index = (int)((angle - publish_msg.angle_min) / publish_msg.angle_increment);
            if(index >= 0 && index < ranges.size()) {
                if(tfFuzzyZero(ranges[alpha]) || std::isnan(ranges[alpha]) || std::isinf(ranges[alpha]))
                {
                  continue;
                }
                else{
                  publish_msg.ranges[index] = ranges[alpha];
                  publish_msg.intensities[index] = intensities[alpha];
                }

            }
        }


        // ??????????????????????????????????????????
        // publish_msg.angle_max = scan_msg->angle_max;
        // publish_msg.angle_min = scan_msg->angle_min - tf::getYaw(start_pose.getRotation()) + tf::getYaw(end_pose.getRotation());
        // publish_msg.angle_increment = (publish_msg.angle_max - publish_msg.angle_min) / (float)(angles.size() - 1);
        // for(int index = ranges.size() - 1; index >= 0; --index) {
        //     publish_msg.ranges.push_back(ranges[index]);
        // }

        // if(fabs(- tf::getYaw(start_pose.getRotation()) + tf::getYaw(end_pose.getRotation()) > 0.01)) {
        //     ROS_INFO("after- angle_min: %f, angle_max: %f, angle_increment: %f", publish_msg.angle_min, publish_msg.angle_max, publish_msg.angle_increment);
        //     ROS_INFO("orign- angle_min: %f, angle_max: %f, angle_increment: %f", scan_msg->angle_min, scan_msg->angle_max, scan_msg->angle_increment);

        //     float range_alpha, angle_alpha, range_beta, angle_beta;
        //     for(int alpha = ranges.size() - 1; alpha >= 0; --alpha) {
        //         range_alpha = ranges[alpha];
        //         angle_alpha = angles[alpha];

        //         int beta = ranges.size() - 1 - alpha;
        //         range_beta = publish_msg.ranges[beta];
        //         angle_beta = publish_msg.angle_min + beta * publish_msg.angle_increment;

        //         ROS_INFO("\t%d\t%f\t%f\t%f\t%f\t", alpha, range_alpha, angle_alpha, range_beta, angle_beta);
        //     }
        // }
        
        scan_pub_.publish(publish_msg);
    }
};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "LidarMotionCalib");

    LidarMotionCalibrator tmpLidarMotionCalib(ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

