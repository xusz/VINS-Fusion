/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
// 不一致
#pragma once
 
#include <thread>
#include <mutex>
//#include <std_msgs/Header.h>
//#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"
#include "draw_result.hpp"
#include "Header.h"

// 对应 Estimator
class Estimator
{
public:

    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &image, const double header);
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    /**
     * vins系统初始化
     * 1. 纯视觉初始化
     * 2. imu与视觉对齐
     */
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    /**
     * 后端非线性优化
     */
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d> > &accVector,
                                              vector<pair<double, Eigen::Vector3d> > &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d> > &accVector);

    enum SolverFlag
    {
        INITIAL,     // 还未成功初始化，进行线性初始化
        NON_LINEAR   // 已成功初始化，正处于紧耦合优化状态，进行非线性优化
    };
    
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,        // marg最老帧
        MARGIN_SECOND_NEW = 1  // marg次新帧
    };
    
    enum InitStatus
    {
        FAIL_START,
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_COST,
        SUCC
    };

    std::mutex mProcess;
    std::mutex mBuf;
    queue<pair<double, Eigen::Vector3d> > accBuf;
    queue<pair<double, Eigen::Vector3d> > gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    double prevTime, curTime;
    bool openExEstimation;   // unused

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    // camera与IMU的外参
    Matrix3d ric[2];   // 从相机到IMU的旋转
    Vector3d tic[2];   // 从相机到IMU的平移
	
    // 当前时刻PVQ，此处的计算值 不带noise
    Vector3d Ps[(WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的位置
    Vector3d Vs[(WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的速度
    Matrix3d Rs[(WINDOW_SIZE + 1)];   // 滑动窗口中各帧在世界坐标系下的旋转
    Vector3d Bas[(WINDOW_SIZE + 1)];  // 滑动窗口中各帧对应的加速度计偏置
    Vector3d Bgs[(WINDOW_SIZE + 1)];  // 滑动窗口中各帧对应的陀螺仪偏置
    double td;   // Sensor 与 Camera 时间差

    // last_R, last_P: 滑窗里面最新的位姿
    // last_R0, last_P0: 滑窗里面最旧的位姿
    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;

    double Headers[(WINDOW_SIZE + 1)];  // Image 的时间戳
    // 针对一个滑动窗口的，其中的每个元素保存的都是两帧之间IMU的预积分数据
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    /**
     * 最近一次接收到的IMU数据，用于下一次IMU积分
     */
    Vector3d acc_0;
    Vector3d gyr_0;
    
    // IMU数据对应的时间间隔
    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    // 加速度计测量值
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    // 陀螺仪测量值
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

	int frame_count;   // 最新帧在滑动窗口中的索引（0，1，2，... ，WINDOW_SIZE）
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager  f_manager;    // 用于管理滑动窗口对应的特征点数据
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;  // unused
	vector<Vector3d> margin_cloud; // unused
    vector<Vector3d> key_poses;    // unused
    /*
     * VINS系统完成初始化操作时对应的图像帧的时间戳
     *（需要注意的是，虽然完成了初始化操作，但是初始化不一定成功）
     */
    double initial_timestamp;

    /**
     * 用于ceres优化的参数块，待优化参数
     */
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];  // p_3 & q_4
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]; // v_3 & ba_3 & bg_3
    double para_Feature[NUM_OF_F][SIZE_FEATURE];   // λ
    double para_Ex_Pose[2][SIZE_POSE];    // 相机Pose：p_3 & q_4
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    /*
     * for initialization 存储相应的预积分和图像特征点信息
     */
    // 存储所有的ImageFrame对象（每读取一帧图像就会构建ImageFrame对象）
    // 键是图像帧的时间戳，值是ImageFrame对象，ImageFrame对象中保存了图像帧的位姿，
    // 相应的预积分和图像特征点信息
    // all_image_frame.size 可能会大于 WINDOW_SIZE
    map<double, ImageFrame> all_image_frame;
    
    // for initialization，用于在创建ImageFrame对象时，
    // 把该指针赋给imageframe.pre_integration
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
    
//////////////////// FOR UI ////////////////////
    DrawResult drawresult {0.0, 0.0, 0.0, 0.0, 0.0, 7.0};
    cv::Mat image_show;
    cv::Mat imageAI;
    
    InitStatus init_status;
    
    bool failure_hand;
    int parallax_num_view;  // Just for log
    int init_fail_cnt;      // Vins初始化失败的次数
    int initProgress;       // 初始化进度(%)
    int feature_num;
    
    // Just for log
    double final_cost;
    double visual_cost;
    int visual_factor_num;
};



/** TODO:
 * pre_integrations 和 tmp_pre_integration 在什么时候用到？
 * 是如何使用的？
 *
 * tmp_pre_integration 只在 all_image_frame中使用，
 * 而all_image_frame 只在 initialization 中用到，
 * 所以 tmp_pre_integration 相关的逻辑需要重新调整
 */
