/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "VinsEntry.h"

VinsEntry::VinsEntry()
{
}

// TODO: 待补充
VinsEntry::~VinsEntry()
{
}

void VinsEntry::startVins(const char * configFile)
{
    std::string _configFile = configFile;
    readParameters(_configFile);
    estimator.setParameter();
}

void VinsEntry::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    estimator.initFirstPose(p, r);
}

void VinsEntry::inputIMU(double t, const Vector3d &acc, const Vector3d &gyr)
{
    estimator.inputIMU(t, acc, gyr);
}

void VinsEntry::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    if (_img1.empty())
    {
        estimator.inputImage(t, _img);
    }
    else
    {
        estimator.inputImage(t, _img, _img1);
    }
}

void VinsEntry::changeSensorType(int use_imu, int use_stereo)
{
    estimator.changeSensorType(use_imu, use_stereo);
}

// 角点帧数据格式：三维齐次坐标*3；二维像素平面坐标*2；x，y方向速度*2；共7个基本数据
void VinsEntry::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &featureFrame)
{
    estimator.inputFeature(t, featureFrame);
}

void VinsEntry::restartVins()
{
    estimator.clearState();
    estimator.setParameter();
}


//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "vins_estimator");
//    ros::NodeHandle n("~");
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
//
//    if(argc != 2)
//    {
//        printf("please intput: rosrun vins vins_node [config file] \n"
//               "for example: rosrun vins vins_node "
//               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
//        return 1;
//    }
//
//    string config_file = argv[1];
//    printf("config_file: %s\n", argv[1]);
//
//    readParameters(config_file);
//    estimator.setParameter();
//
//#ifdef EIGEN_DONT_PARALLELIZE
//    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
//#endif
//
//    ROS_WARN("waiting for image and imu...");
//
//    registerPub(n);
//
//    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
//    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
//    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
//    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
//    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
//    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
//
//    std::thread sync_thread{sync_process};
//    ros::spin();
//
//    return 0;
//}

