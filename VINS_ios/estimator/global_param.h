//
//  global_param.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#pragma once

#include "parameters.h"

#define DEBUG_MODE  false

enum DeviceType
{
    iPhone5s,
    iPhone7P,
    iPhone7,
    iPhone6P,
    iPhone6sP,
    iPhone6s,
    iPadPro97,
    iPadPro129,
    unDefine
};

// extrinsic param
#define RIC_y  ((double)0.0)
#define RIC_p  ((double)0.0)
#define RIC_r  ((double)180.0)
//double RIC_y = ((double)0.0);
//double RIC_p = ((double)0.0);
//double RIC_r = ((double)180.0);

#define MIN_LOOP_NUM   22
#define LOOP_FREQ      3
//#define WINDOW_SIZE    10  // 滑动窗口的大小
#define PNP_SIZE       6

#define SIZE_POSE      7   // 7 DoF(x, y, z, qx, qy, qz, qw)
// 9 DoF(vx,vy,vz, bas_x,bas_y,bas_z, bgs_x,bgs_y,bgs_z)
#define SIZE_SPEEDBIAS 9
#define SIZE_SPEED     3   // 3 DoF(vx, vy, vz)
#define SIZE_BIAS      6   // 6 DoF(bas_x,bas_y,bas_z, bgs_x,bgs_y,bgs_z)
#define SIZE_FEATURE   1   // 1 DoF(inv_depth)

//#define NUM_OF_F       1000 // TODO: 对于手机端，特征点是不是太多了？？？
//#define NUM_OF_CAM     1
#define C_PI           3.1415926

#define GRAVITY ((double)9.805)

// accelerometer measurement noise standard deviation. #0.2
//#define ACC_N   ((double)0.5)
//// accelerometer bias random work noise standard deviation. #0.02
//#define ACC_W   ((double)0.002)
//// gyroscope measurement noise standard deviation. #0.05
//#define GYR_N   ((double)0.2)
//// gyroscope bias random work noise standard deviation. #4.0e-5
//#define GYR_W   ((double)4.0e-5)

#define BIAS_ACC_THRESHOLD  ((double)0.5)
#define BIAS_GYR_THRESHOLD  ((double)0.1)
#define G_THRESHOLD         ((double)3.0)
#define G_NORM              ((double)9.805)   // gravity magnitude  #9.81007
#define INIT_KF_THRESHOLD   ((double)18)
#define SFM_R_THRESHOLD     ((double)180)
// #define MAX_FEATURE_CNT 150

extern double FOCAL_LENGTH_Y;
extern double PY;
extern double FOCAL_LENGTH_X;
extern double PX;
extern double SOLVER_TIME;
extern int FREQ;

//extrinsic param
extern double TIC_X;
extern double TIC_Y;
extern double TIC_Z;
/* IMU
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 
 */
//enum StateOrder
//{
//    O_P = 0,
//    O_R = 3,
//    O_V = 6,
//    O_BA = 9,
//    O_BG = 12
//};

bool setGlobalParam(DeviceType device);

#if 1
    #define TS(name) int64 t_##name = cv::getTickCount()
    #define TE(name) printf("TIMER_" #name ": %.2fms\n", \
                1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
#else
    #define TS(name)
    #define TE(name)
#endif

