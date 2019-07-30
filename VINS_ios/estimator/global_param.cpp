//
//  global_param.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/05/09.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//
// TODO: 不一致，需要和paramters.cpp 合并
#include <stdio.h>
#include "global_param.h"
#include "parameters.h"

double FOCAL_LENGTH_Y;  // fx 参数
double PY;
double FOCAL_LENGTH_X;  // 焦距
double PX;              // ppx 参数
//double SOLVER_TIME;

/**
 * // 控制发布次数的频率 frequence (Hz) of publish tracking result.
 * At least 10Hz for good estimation. If set 0, the frequence will be same as raw image
 */
int FREQ;

// extrinsic param
double TIC_X;
double TIC_Y;
double TIC_Z;

bool setGlobalParam(DeviceType device)
{
    switch (device) {
        case iPhone7P:
            printf("Device iPhone7 plus param\n");
            FOCAL_LENGTH_X = 526.600;
            FOCAL_LENGTH_Y = 526.678;
            PX = 243.481;
            PY = 315.280;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            return true;
            break;
            
        case iPhone7:
            printf("Device iPhone7 param\n");
            FOCAL_LENGTH_X = 526.958;
            FOCAL_LENGTH_Y = 527.179;
            PX = 244.473;
            PY = 313.844;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            return true;
            break;
            
        case iPhone6s:
            printf("Device iPhone6s param\n");
            FOCAL_LENGTH_Y = 549.477;
            PY = 320.379;
            FOCAL_LENGTH_X = 548.813;
            PX = 238.520;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            return true;
            break;
            
        case iPhone6sP:
            printf("Device iPhone6P param\n");
            FOCAL_LENGTH_X = 547.565;
            FOCAL_LENGTH_Y = 547.998;
            PX = 239.033;
            PY = 309.452;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            return true;
            break;
            
        case iPhone6P:
            printf("Device iPhone6sP param\n");
            FOCAL_LENGTH_X = 526.600;
            FOCAL_LENGTH_Y = 526.678;
            PX = 243.481;
            PY = 315.280;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.01;
            return true;
            break;
            
        case iPhone5s:
            printf("Device iPhone5s param\n");
            FOCAL_LENGTH_Y = 549.477;
            PY = 320.379;
            FOCAL_LENGTH_X = 548.813;
            PX = 238.520;
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.065;
            TIC_Z = 0.0;
            return true;
            break;
            
        case iPadPro97:
            printf("Device ipad97 param\n");
            FOCAL_LENGTH_X = 547.234;
            FOCAL_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            return true;
            break;
            
        case iPadPro129:
            printf("Device iPad129 param\n");
            FOCAL_LENGTH_X = 547.234;
            FOCAL_LENGTH_Y = 547.464;
            PX = 241.549;
            PY = 317.957;
            
            SOLVER_TIME = 0.06;
            FREQ = 3;
            
            // extrinsic param
            TIC_X = 0.0;
            TIC_Y = 0.092;
            TIC_Z = 0.1;
            return true;
            break;
        case unDefine:
            return false;
            break;
        default:
            return false;
            break;
    }
}

