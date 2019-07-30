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
// 和Mobile不一致
#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "global_param.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include <set>
#include <map>

//#define MAX_CNT      70    // 特征点最大个数  原值：150
//#define MIN_DIST     30    // 特征点之间的最小间隔
//#define COL          480   // 图像宽度
//#define ROW          640   // 图像高度
//#define F_THRESHOLD  1.0   // ransac算法的门限  ransac threshold (pixel)
///**
// * 光太亮或太暗则为1，进行直方图均衡化
// * if image is too dark or light, trun on equalize to find enough features
// */
//#define EQUALIZE    1


using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    // 角点帧数据格式：三维齐次坐标*3；二维像素平面坐标*2；x，y方向速度*2；共7个基本数据
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;  // 图像掩码
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;     // 每一帧中新提取的特征点
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;  // 像素平面当前x坐标
    // 从上面的*_pts 中找到畸变纠正后的点*_un_pts；归一化相机坐标系下的坐标
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;  // undistorted Pts 畸变纠正后的点
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    
    vector<int> ids, ids_right;     // 正在处理的帧 能够被跟踪到的特征点的id
    // 代表当前cur_ptrs被追踪的时间次数
    vector<int> track_cnt;  // 当前帧forw_img中每个特征点被追踪的时间次数
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;   // For draw track
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    /*
     * 用来作为特征点id，每检测到一个新的特征点，就将n_id作为该特征点的id，然后n_id加1
     */
    int n_id;
    bool hasPrediction;
};


