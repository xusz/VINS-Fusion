/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>
#include "VinsLog.h"

using namespace std;

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


const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR  //！求取切平面上的误差

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    // 7 DoF(x, y, z, qx, qy, qz, qw)
    SIZE_POSE = 7,
    // 9 DoF(vx,vy,vz, bas_x,bas_y,bas_z, bgs_x,bgs_y,bgs_z)
    SIZE_SPEEDBIAS = 9,
    // 1 DoF(inv_depth)
    SIZE_FEATURE = 1
};

/* IMU
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 
 */
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

