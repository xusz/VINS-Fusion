
#pragma once

#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "Imu.h"
#include "Image.h"
#include "PointCloud.h"

using namespace std;

class VinsEntry
{
public:
    
    VinsEntry();
    ~VinsEntry();
    
    void startVins(const char * configFile);
    
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    
    // 角点帧数据格式：三维齐次坐标*3；二维像素平面坐标*2；x，y方向速度*2；共7个基本数据
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &featureFrame);
    
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    
    void changeSensorType(int use_imu, int use_stereo);
    
    void restartVins();
    
public:
    Estimator estimator;
};

