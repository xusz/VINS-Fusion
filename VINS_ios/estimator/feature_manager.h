/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <set>
#include <map>
using namespace std;

#include <eigen3/Eigen/Dense>
#include "global_param.h"
#include "utility.h"

using namespace Eigen;

#include "../utility/tic_toc.h"

#define COMPENSATE_ROTATION  false
#define MIN_PARALLAX_POINT  ((double)(3.0/549))
#define MIN_PARALLAX        ((double)(10.0/549))
#define INIT_DEPTH          ((double)(5.0))

enum SOLVE_FLAG {
    SOLVE_NONE = 0,  // haven't solve yet
    SOLVE_SUCC,      // solve succ
    SOLVE_FAIL       // solve fail;
};

// 特征点在某个图像帧下的坐标（归一化平面坐标）
class FeaturePerFrame
{
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    bool is_stereo;
};

class FeaturePerId
{
public:
    const int feature_id;
    int start_frame;
    
    // feature_id 在哪些帧中被观察到
    vector<FeaturePerFrame> feature_per_frame;
    
    int used_num;     // 此特征点被多少frame观测到，不需要作为成员使用

    double estimated_depth;
    
    // SOLVE_FLAG : 0 haven't solve yet; 1 solve succ; 2 solve fail;
    int solve_flag;
    
    FeaturePerId(int _feature_id, int _start_frame)
                : feature_id(_feature_id),
                  start_frame(_start_frame),
                  used_num(0),
                  estimated_depth(-1.0),
				  solve_flag(SOLVE_NONE)
    {
    }
    
    int endFrame();
    
    // 是否可作为先验特征，被观测到的次数>=2次 && 起始帧不在最后两帧
    inline bool isPriorFeature()
    {
        return (used_num >= 2 && start_frame < WINDOW_SIZE - 2);
    }
};

class FeatureManager
{
public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    
    void clearState();
    
    int getFeatureCount();
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &image, double td);
    vector<pair<Vector3d, Vector3d> > getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(const int frameCnt, const Vector3d Ps[], const Matrix3d Rs[], const Vector3d tic[], const Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);

    list<FeaturePerId> feature;   // 管理滑动窗口中所有的特征点
    int last_track_num;           // 最新帧图像跟踪到的特征点的数量
//    double last_average_parallax;
//    int new_feature_num;
//    int long_track_num;

private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

    const Matrix3d *Rs;
    Matrix3d ric[2];     // unused
};

#endif

