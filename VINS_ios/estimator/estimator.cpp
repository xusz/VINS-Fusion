/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
// 不一致
#include "estimator.h"

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    initProgress = 0;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
//        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    
    f_manager.setRic(ric);  // unused
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc featureTrackerTime;

    inputImageCnt++;
    // 角点帧数据格式：三维齐次坐标*3；二维像素平面坐标*2；x，y方向速度*2；共7个基本数据
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > featureFrame;

    if (_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    
    printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
//        pubTrackImage(imgTrack, t);
    }

    if (MULTIPLE_THREAD)  // TODO: 多线程实现逻辑可能有误
    {
        if (inputImageCnt % 2 == 0)  // TODO: why
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("++++++ process time: %f\n", processTime.toc());
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    fastPredictIMU(t, linearAcceleration, angularVelocity);
//    if (solver_flag == NON_LINEAR)
//        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if (!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d> > &accVector,
                                vector<pair<double, Eigen::Vector3d> > &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        // 角点帧数据格式：三维齐次坐标*3；二维像素平面坐标*2；x，y方向速度*2；共7个基本数据
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d> > accVector, gyrVector;
        if (!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.first + td;
            while (1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if (USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            featureBuf.pop();
            mBuf.unlock();

            if (USE_IMU)
            {
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mProcess.lock();
            processImage(feature.second, feature.first);
            
            prevTime = curTime;  // prevTime 放在 processImage 之后

//            printStatistics(*this, 0);

            // TOOD: publish计算完毕后的数据给UI
//            std_msgs::Header header;
//            header.frame_id = "world";
//            header.stamp = ros::Time(feature.first);
//
//            pubOdometry(*this, header);
//            pubKeyPoses(*this, header);
//            pubCameraPose(*this, header);
//            pubPointCloud(*this, header);
//            pubKeyframe(*this);
//            pubTF(*this, header);
            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d> > &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

/**
 * 1 当滑窗不满的时候，把当前测量值加入到滑窗指定位置
 * 2 残差和雅可比矩阵、协方差矩阵保存在pre_integrations中
 *      propagate (_dt[时间间隔]; _acc_1 [加速度计数据]; _gyr_1 [陀螺仪数据])
 *      里面主要为 midPointIntegration 函数即:   中值法进行预积分
 *      得到状态变化量result_delta_q，result_delta_p，result_delta_v，
 *      result_linearized_ba，result_linearized_bg和得到更新的协方差矩阵和雅可比矩阵
 * 3 提供imu计算的当前旋转，位置，速度，作为优化的初值
 *
 * @param dt 是当前IMU和前一个IMU的时间差
 * @param linear_acceleration 为当前加速度计数据
 * @param angular_velocity 为当前角速度计数据
 */
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    // clearState() 后 first_imu = false
    if (!first_imu)  // 未获取第一帧IMU数据
    {
        first_imu = true;
        
        // 将第一帧IMU数据记录下来
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    
    // 调用imu的预积分，计算对应的状态量、协方差和雅可比矩阵；注意frame_count参数的作用
    // 用中值法进行预积分
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    
    // 每次pre_integration 推入一个IMU数据后就会进行积分：
    if (frame_count != 0) // 在初始化时，第一帧图像特征点数据没有对应的预积分
    {
        // covariance propagate  协方差传播
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

//        if (solver_flag != NON_LINEAR)  // 还未初始化完成
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 将时间、加速度和角速度分别存入相应的缓存中
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        /**
         * IMU数据进行积分，得到当前时刻的PVQ，使用midpoint integration，对应5.3节
         *
         * 当积完一个measurement中所有IMU数据后，就得到了对应frame在[世界坐标系]中的Ps、Vs、Rs
         *
         * 下面这部分的积分，在未完成初始化时似乎是没有意义的，因为未完成初始化时，对IMU数据来说是没有世界坐标系的
         * 当完成初始化后，下面的积分才有用，它可以通过IMU积分得到滑动窗口中最新帧在世界坐标系中的PVQ
         */
        int j = frame_count;
        // 下面都采用的是中值积分的传播方式， noise是zero mean Gassu，在这里忽略了
        
        // 对应5.3节中的公式（5-13）的前半部分
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        
        // 对应5.3节中的公式（5-14）
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        // 各帧在世界坐标系中的旋转，对应5.3节中的公式（5-12）
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        
        // 对应5.3节中的公式（5-13）的后半部分
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        // 对应5.3节中的公式（5-13）
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        
        // 各帧在世界坐标系中的位移，对应5.3节中的公式（5-10）
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        // 各帧在世界坐标系中的速度，对应5.3节中的公式（5-11）
        Vs[j] += dt * un_acc;
    }
    
    // 保留此次接收到的IMU数据，用于下一次IMU数据积分
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/*
 * processImage
 * 当处理完一批imu_msg后，在process函数中就会紧接着处理图像数据,
 * 当图像数量达到窗口大小时，在optimization函数中就会把IMU误差项加进去进行优化，
 * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
 * image_msg -- point_clouds，当前帧的点云数据
 */
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    
    // 对相近的特征点进行视差计算
    /* 通过检测两帧之间的视差决定是否作为关键帧，
     * 同时添加之前检测到的特征点到feature（list< FeaturePerId >）这个容器中，
     * 计算每一个点跟踪的次数，以及它的视差
     */
    // 通过检测两帧之间的视差决定是否作为关键帧，同时添加之前检测到的特征点到feature容器中，
    // 计算每一个点跟踪的次数，以及它的视差
    /*
     * 把当前帧图像（frame_count）的特征点添加到f_manager.feature容器中
     * 计算第2最新帧与第3最新帧之间的平均视差（当前帧是第1最新帧），
     *      然后判断是否把第2最新帧添加为关键帧
     * 在未完成初始化时，如果窗口没有塞满，那么是否添加关键帧的判定结果不起作用，滑动窗口要塞满
     * 只有在滑动拆个纽扣塞满后，或者初始化完成之后，才需要滑动窗口，此时才需要做关键帧判别，
     *      根据第2最新关键帧是否未关键帧选择相应的边缘化策略
     */
    // 向Featuresmanger中添加Features并确定共视关系及视差角的大小
    /** 判断该帧是否关键帧
     * 关键帧的判断依据是rotation-compensated过后的parallax足够大，
     * 并且tracking上的feature足够多；关键帧会保留在当前Sliding Window中，
     * marginalize掉Sliding Window中最旧的状态，如果是非关键帧则优先marginalize掉
     */
    // frame_count < WINDOW_SIZE的时候，在后续的slideWindow中并没有执行marg
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    // all_image_frame:for initialization 存储相应的预积分和图像特征点信息
    all_image_frame.insert(make_pair(header, imageframe));
    // 为下一帧frame及IMU做准备
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d> > corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            // TODO: 需要再看一遍
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
//                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);

                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)  // 需要初始化，进行线性初始化
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            // 滑动窗口中塞满了才进行初始化
            if (frame_count == WINDOW_SIZE)
            {
                // TODO: 不一致
//                if (track_num < 20)
//                {
//                    clearState();
//                    return;
//                }
                
                bool result = false;
                
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) // 0.3
                {
                    // vins系统初始化
                    result = initialStructure();
                    initial_timestamp = header;
                }
                
                if (result)  // 初始化成功
                {
                    failure_occur = 0;
                    initProgress = 100;
                    init_status = SUCC;
                    init_fail_cnt = 0;
                    
                    solver_flag = NON_LINEAR;
                    optimization();
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();  // 初始化不成功，对窗口进行滑动
            }
        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                solver_flag = NON_LINEAR;
                optimization();
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if (STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // 滑动窗口没塞满，接着塞
        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }
    else  // solver_flag == NON_LINEAR 进行非线性优化
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();   
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();  // TODO:
    }  
}

/**
 * vins系统初始化
 * 1.确保IMU有足够的excitation
 * 2.检查当前帧（滑动窗口中的最新帧）与滑动窗口中所有图像帧之间的特征点匹配关系，
 *   选择跟当前帧中有足够多数量的特征点（30个）被跟踪，且由足够视差（20 pixels）的某一帧，利用五点法恢复相对旋转和平移量。
 *   如果找不到，则在滑动窗口中保留当前帧，然后等待新的图像帧
 * 3.sfm.construct 全局SFM 恢复滑动窗口中所有帧的位姿，以及特征点三角化
 * 4.利用pnp恢复其它帧
 * 5.visual-inertial alignment：视觉SFM的结果与IMU预积分结果对齐
 * 6.给滑动窗口中要优化的变量一个合理的初始值以便进行非线性优化
 */
bool Estimator::initialStructure()
{
    TicToc t_sfm;

    // check imu observibility, 通过协方差检测IMU的可观测性
    // 保证IMU充分运动，通过线加速度判断，一开始通过线加速度的标准差（离散程度）判断保证IMU充分运动，加速度标准差大于0.25则代表imu充分激励，足够初始化。
    // TODO: 为什么注释掉？？？ if(var < 0.25) return false;
    // TODO: 是否需要全部注掉？？？
//    {
//        map<double, ImageFrame>::iterator frame_it;
//        Vector3d sum_g;
//        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
//        {
//            double dt = frame_it->second.pre_integration->sum_dt;
//            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
//            sum_g += tmp_g;
//        }
//        Vector3d aver_g;
//        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
//        double var = 0;
//        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
//        {
//            double dt = frame_it->second.pre_integration->sum_dt;
//            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
//            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
//            //cout << "frame g " << tmp_g.transpose() << endl;
//        }
//        var = sqrt(var / ((int)all_image_frame.size() - 1));
//        //ROS_WARN("IMU variation %f!", var);
//        if(var < 0.25)
//        {
//            ROS_INFO("IMU excitation not enouth!");
//            //return false;
//        }
//    }
    
    // global sfm
    // 滑动窗口中每一帧的姿态，旋转四元数
    Quaterniond Q[frame_count + 1];
    // 滑动窗口中每一帧的位置
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    
    // 特征点对应的3D feature
    // 用于视觉初始化的图像特征点数据
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        
        SFMFeature tmp_feature;
        tmp_feature.state = false;  // 该特征点的初始状态为：未被三角化
        tmp_feature.id = it_per_id.feature_id;
        
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;   // 观测到该特征点的图像帧的帧号
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    
    /**
     * 在窗口内选择跟最后一帧视差最大的帧，利用五点法计算相对旋转和平移量
     */
    Matrix3d relative_R;  // 从最新帧到选定帧的旋转（推测的）
    Vector3d relative_T;  // 从最新帧到选定帧的位移（推测的）
    int l;                // 选定帧在滑动窗口中的帧号
    
    // 2. 选择跟最新帧中有足够数量的特征点和视差的某一帧，利用五点法恢复相对旋转和平移量
    // 相对旋转和平移量如果找不到，则初始化失败，第l帧作为初始状态帧
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    
    // update init progress
//    initProgress = 30;
    
    // 三角化：已知两帧的相对位姿，求解帧中地标点的3D坐标
    /**
     * sfm.construct 函数本身就很巨大，里面很多步骤，大致如下：
     * 1. 三角化 l 帧与frame_num - 1帧，得到一些地标点；
     * 2. 通过这些地图点，利用pnp的方法计算l+1, l+2, l+3, …… frame_num-2 相对于l的位姿；
     *           而且每计算一帧，就会与frame_num - 1帧进行三角化，得出更多地标点；
     * 3. 三角化l+1, l+2 …… frame_num-2帧与l帧；
     * 4. 对l-1, l-2, l-3 等帧与sfm_f的特征点队列进行pnp求解，得出相对于l的位姿，并三角化其与l帧；
     * 5. 三角化剩余的点（某些地标点仅存在与某些帧中，而非在[l,WINDOW]帧中都存在的地标点）；
     * 6. ceres全局BA优化，最小化3d投影误差；
     */
    
    // 3. 全局SFM初始化滑动窗口中全部初始帧的相机位姿和特征点空间3D位置
    // 3D的feature point和sliding window中的keyFrame的2D feature求解PnP，并且使用ceres优化：
    GlobalSFM sfm;   // 第l帧作为初始状态帧
    if(!sfm.construct(frame_count + 1, Q, T, l,
                      relative_R, relative_T,
                      sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");

        marginalization_flag = MARGIN_OLD;
        
        init_status = FAIL_SFM;
        init_fail_cnt++;

        return false;
    }
    // update init progress
//    initProgress = 50;
    
    // solve pnp for all frame
    // 因为有continue存在，怎么会solve all，感觉只是不满足时间戳相等的才pnp求解
    //
    // 将相机坐标系转换到IMU坐标系中，然后再一次进行PnP求解，3D特征点还是使用之前SFM中求解出来的，后续也没有进行优化
    //
    // 4. 对于非滑动窗口的所有帧，提供一个初始的R,T，然后solve pnp求解pose
    // 由于并不是第一次视觉初始化就能成功，此时图像帧数目有可能会超过滑动窗口的大小
    // 所以在视觉初始化的最后，需要求出滑动窗口以外的帧的位姿
    // 最后把世界坐标系从帧l的相机坐标系，转到帧l的IMU坐标系
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        // all_image_frame与滑动窗口中对应的帧
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;  // 滑动窗口中所有帧都是关键帧
            // 根据各帧相机坐标系的姿态和外参，得到用各帧IMU坐标系的姿态
            //（对应VINS Mono论文(2018年的期刊版论文)中的公式（6））
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        
        // 为滑动窗口外的帧提供一个初始位姿
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        // 罗德里格斯（Rodrigues）旋转向量与矩阵的变换，旋转向量（1x3）与旋转矩阵（3x3）
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        
        // 初始化时，位于滑动窗口外的帧是非关键帧
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;  // 用于pnp解算的3D点
        vector<cv::Point2f> pts_2_vector;  // 用于pnp解算的2D点
        
        // 对于该帧中的特征点
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                // 如果it不是尾部迭代器，说明在sfm_tracked_points中找到了相应的3D点
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    // 记录该id特征点的3D位置
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    
                    // 记录该id的特征点在该帧图像中的2D位置
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        
        // 如果匹配到的3D点数量少于6个，则认为初始化失败
        if(pts_3_vector.size() < 6)
        {
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        
        // TODO: 初始化用的frame是不是会越来越多，导致失败的概率增高，是否可以删除一些旧的frame
        // 所有的frame求解PnP（只要有一帧失败就认为fail，是否合理？）
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            init_status = FAIL_PNP;
            init_fail_cnt++;
            
            return false;
        }
        
        // pnp求解成功
        // 罗德里格斯（Rodrigues）旋转向量与矩阵的变换，旋转向量（1x3）与旋转矩阵（3x3）
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
        R_pnp = tmp_R_pnp.transpose();
        
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        
        // 根据各帧相机坐标系的姿态和外参，得到用各帧IMU坐标系的姿态。
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    
    // update init progress
    initProgress = 75;
    
//    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(), Ps[0].y(), Ps[0].z());
    
    // camera与IMU对齐
    if (visualInitialAlign())
    {
        return true;
    }
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        init_status = FAIL_ALIGN;
        init_fail_cnt++;
        return false;
    }
}


/**
 * cameara与IMU对齐
 */
// Ps:世界坐标下的平移
// Rs:世界坐标下的旋转
// Vs:世界坐标下的速度

// vision IMU数据对齐
// 这里涉及的量有： 陀螺仪的Bias(加速度Bias这里没有处理) 速度V[0:n] 重力g 尺度s
// 更新了Bgs后，则对于imu数据需要repropogate，也就是从当前时刻开始预积分，
//      同时通过三角化和上一步计算的scale可以获得每个feature的深度;
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // solve scale
    /** 里面包含
     * 1. solveGyroscopeBias()零偏初始化: 在滑动窗口中，每两帧之间的相对旋转
     *    与IMU预积分产生的旋转进行最小二乘法优化，用ldlt求解
     * 2. LinearAlignment() 尺度初始化:由于在视觉初始化SFM的过程中，将其中位姿变化较大的两帧中使用E矩阵
     *    求解出来旋转和位移，后续的PnP和三角化都是在这个尺度下完成的。所以当前的尺度与IMU测量出来的
     *    真实世界尺度肯定不是一致的，所以需要这里进行对齐。这里对齐的方法主要是通过在滑动窗口中每两帧之间
     *    的位置和速度与IMU预积分出来的位置和速度组成一个最小二乘法的形式，然后求解出来
     * 3. RefineGravity()重力向量优化:进一步细化重力加速度，提高估计值的精度，
     *    形式与LinearAlignment()是一致的，只是将g改为g⋅ĝ +w1b1+w2b2
     * 2和3的公式相对难懂，请参考下面图片 https://images2018.cnblogs.com/blog/1072373/201804/1072373-20180419163252748-810017249.png
     */
    /*
     * IMU陀螺仪零偏初始化；主要是通过滑动窗口中，每两帧之间通过SFM求解出来的旋转与
     *         IMU预积分的旋转量组成一个最小二乘法形式的等式，求解出来陀螺仪的零偏。
     * 完成视觉SFM的结果与IMU预积分结果对齐
     */
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        // 滑动窗口中各图像帧在世界坐标系下的旋转和平移
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        // 滑动窗口中所有初始帧都是关键帧
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    /**
     * 更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
     */
    double s = (x.tail<1>())(0);
    
    // TODO: 在VisualIMUAlignment中已经执行过repropagate了，此处为什么还执行一次？？？
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    
    // 在VisualIMUAlignment中以第l帧为参考帧，此处转化到以第一帧为参考帧
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    // 当前参考坐标系与世界坐标系（依靠g构建的坐标系）的旋转矩阵，暂时没搞清楚从谁转到谁？？
    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    
    // 更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
    for (int i = 0; i <= frame_count; i++)
    {
        // 似乎是把Ps、Rs、Vs转到世界坐标系下
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
//    ROS_DEBUG_STREAM("g0     " << g.transpose());
//    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    // 计算feature的depth
    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

// 判断两帧有足够视差，在滑窗中寻找与最新的关键帧共视关系较强的关键帧
// 在滑动窗口中，寻找与最新帧有足够多数量的特征点对应关系和视差的帧，
// 然后用5点法恢复相对位姿，由对极约束中的F矩阵恢复出相对的R、t
/**
 * relativePose方法中首先通过FeatureManeger获取（滑动窗口中）第i帧和最后一帧的特征匹配corres，
 * 当corres匹配足够大时，考察最新的keyFrame和slidingwindow中某个keyFrame之间有足够
 * feature匹配和足够大的视差（id为l=i），满足这两个条件，然后这两帧之间通过五点法恢复出R，t
 * 并且三角化出3D的特征点feature point，这里是使用solveRelativeRT
 *
 * 这里值得注意的是，这种relativePose得到的位姿是第l帧的，第l帧的筛选是从第一帧开始到
 * 滑动窗口所有帧中一开始满足平均视差足够大的帧，这里的第l帧会作为参考帧到下面的全局SFM使用。
 */
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d> > corres;
        // getCorresponding::WINDOW_SIZE 是最后一帧，也即最新帧
        // 第i帧与最新帧之间的Feature共视
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        // 滑窗内两帧的共视
        // 共视的Features应该大于30
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            
            // 求取所有匹配的特征点的平均视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            
            // 视差大于一定阈值，并且能够有效地求解出变换矩阵，找到的关键帧设置为l帧
            // solveRelativeRT 利用cv::findFundamentalMat 计算基础矩阵用5点法来得到变换矩阵
            // 利用五点法求解相机初始位姿  TODO: 进一步理解solveRelativeRT
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                init_fail_cnt++;
                return false;
            }
        }
    }
    
    return false;
}


// 数据转换，因为ceres使用数值数组
// （猜测）将PVQ等赋值给待优化参数para_*，在ceres优化过程中作为初始值
void Estimator::vector2double()  // old2new
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

/**
 * 数据转换，因为ceres使用数值数组，vector2double的相反过程
 */
// （猜测）ceres将待优化参数para_*优化完毕后，赋值给PVQ等
void Estimator::double2vector()  // new2old
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;  // TODO:
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).toRotationMatrix();
        }
    }
    // 赋值λ
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];

}

/**
 * 检测SLAM系统是否失败
 */
bool Estimator::failureDetection()
{
//    return false;  // TODO: 不一致，完全无用
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}


/**
 * 基于滑动窗口的紧耦合的后端非线性优化，对应第八章
 *
 * 在solve_ceres函数中就会把IMU误差项加进去进行优化，
 * 在这之前有一些边缘化的操作，而且这个操作会影响pre_integrations数组
 */
// 1. 添加要优化的变量，也就是滑动窗口的位置para_Pose[0:n]
//    速度和Bias para_SpeedBias[0:n]一共15个自由度，IMU的外参也可以加进来估计
// 2. 添加残差，残差项分为4块 先验残差+IMU残差+视觉残差+闭环检测的残差
// 3. 根据倒数第二帧是不是关键帧确定marginization的结果，下面有详细解释
//
// 16.9节 后端优化之optimazation
void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    
    /**
     * 因为ceres用的是double数组，所以在下面用vector2double做类型转换
     * （猜测，用转换后的值作为优化过程中的初值）
     * Ps,Rs 转换为 para_Pose
     * Vs,Bas,Bgs 转换为 para_SpeedBias
     * tic,ric 转换为 para_Ex_Pose
     *
     * 把要优化的变量转成数组的形式
     */
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;  // TODO: 不一致
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    // 添加需要优化的变量（camera的pose，imu的biases）
    //！Step1.1：添加待优化状态量 [p,q](7)，[speed,ba,bg](9)
    // 添加sliding window frame的state，(p,v,q,ba,bg),
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        // SIZE_POSE(7) compose px py pz, qx qy qz qw
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)  // SIZE_SPEEDBIAS(9) compose vx vy vz bax bay baz bgx bgy bgz
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    
    /**
     * TODO: 函数中所有 new 的对象，是不是在函数结束的时候应该delete掉？？？
     */
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    //！Step1.2：添加camera与IMU的外参估计[p_cb,q_cb](7)
    // 添加camera和IMU的坐标变换的变量
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // 令指定的参数块在整个优化计算过程中保持常数
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    // 是否估计TD: 令指定的参数块在整个优化计算过程中保持常数
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    // marginalization factor 边缘化因子
    // 添加marginalization的residual，这个marginalization的结构是始终存在的，
    // 随着下面marginazation的结构更新，last_marginalization_parameter_blocks
    // 对应的是还在sliding window的变量
    // 关于这一部分的理解http://blog.csdn.net/heyijia0327/article/details/53707261
    // 这里可以这样理解，下面会添加对IMU和视觉的残差，但是，这些对应的变量实际上跟之前
    // 被margin掉的变量是有约束的，这里的last_marginalization_parameter_blocks
    // 就是保存的这些变量，也就是heyijia博客中对应的Xb变量，
    // last_marginalization_info中对应的是Xb对应的测量Zb，
    // 这里用先验来表示这个约束，整个margin部分实际上就是在维护这个结构：
    /**
     * add residual for prior from last marginalization
     * 添加边缘化残差约束：1个
     */
    //！Step1.3：添加滑窗残差约束
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    /**
     * 添加IMU残差约束：WINDOW_SIZE个, 每相邻两个Pose之间一个IMU residual项
     */
    if (USE_IMU)
    {
        // 一个IMUfactor误差项对应着一个pre_integration实例，而且计算的是两个帧之间IMU的误差
        // 这里IMU项和camera项之间是有一个系数，这个系数就是它们各自的协方差矩阵：IMU的协方差
        // 是预积分的协方差(IMUFactor::Evaluate，中添加IMU协方差，求解jacobian矩阵)，
        // 而camera的则是一个固定的系数（f/1.5）
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;   // pre_integrations[ i+1 ]
            if (pre_integrations[j]->sum_dt > 10.0)  // TODO: 不一致
                continue;
            
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    
    // projection factor 视觉投影因子
    int f_m_cnt = 0;
    int feature_index = -1;
    
    /**
     * ！Step1.4：添加视觉残差约束，add residual for per feature to per frame
     * 添加视觉残差约束：被观测数大于2的特征, 首次观测与后面的每次观测之间各一个residual项
     */
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
//        if (!it_per_id.isPriorFeature())
        if (it_per_id.used_num < 4)  // TODO: 不一致
            continue;
        
        ++feature_index;

        // 下一帧，对应后面的imu_j++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());
    // 设置 ceres 属性  TODO: 不一致
    ceres::Solver::Options options;
    
    // TODO: ceres::SPARE_SCHUR 是否更快，精度如何？？？
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // # max solver itrations, to guarantee real time
    options.max_num_iterations = NUM_ITERATIONS;
    
//    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    // options.use_nonmonotonic_steps = true;
    
    // 最大求解时间 (TODO: 此处设置的时间内，未必能求解完)
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    
    TicToc t_solver;
    ceres::Solver::Summary summary;  // 解算过程报告输出，可以不设置输出？？？
    /*
     * IMU误差的Jacobian矩阵的计算,
     * 这里就使用到了pre_integration实例里面的Jacobian的部分结果，
     * Jacobian数组里每一项都是IMU误差关于两帧图像状态的导数，只不过这里把pose和speedBias分开了
     */
    // 约束求解
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    // （猜测）ceres::Solve 将待优化参数para_* 优化完毕后，赋值给PVQ等
    double2vector();

    // TODO: 是不是需要清理资源？？？
//    vector<ceres::ResidualBlockId> residual_set;
//    problem.GetResidualBlocks(&residual_set);
//    for (auto it : residual_set)
//    {
//        problem.RemoveResidualBlock(it);
//    }
    
    // TODO: frame_count会小于WINDOW_SIZE吗？？？
    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
	/*
     * Step3:marg部分
     *  3.1 对于边缘化首帧
     *    3.1.1 把之前存的残差部分加进来
     *    3.1.2 把与首帧相关的残差项加进来,包含IMU,vision.
     *    3.1.3 计算所有残差项的残差和雅克比
     *    3.1.4 多线程构造Hx=b的结构
     *    3.1.5 marg结束,调整参数块在下一次window的位置
     *  3.2 对于边缘化倒数第二帧
     *    3.2.1 如果倒数第二帧不是关键帧,保留该帧的IMU测量,去掉该帧的visual,代码中都没写.
     *    3.2.2 计算所有残差项的残差和雅克比
     *    3.2.3 多线程构造Hx=b的结构,(需要细看)
     *    3.2.4 marg结束,调整参数块在下一次window的位置
     */
    // for marginalization back
    // margin部分，如果倒数第二帧是关键帧：
    // 1.把之前存的残差部分加进来
    // 2.把与当前要margin掉帧所有相关的残差项都加进来，IMU,vision
    // 3.preMarginalize-> 调用Evaluate计算所有ResidualBlock的残差，parameter_block_data parameter_block_idx parameter_block_size是marinazation中存参数块的容器(unordered_map),key都是addr,
    //分别对应这些参数的data，在稀疏矩阵A中的index(要被margin掉的参数会被移到前面)，A中的大小
    // 4.Marginalize->多线程构造Hx=b的结构，H是边缘化后的结果，First Esitimate Jacobian,在X_0处线性化
    // 5.margin结束，调整参数块在下一次window中对应的位置（往前移一格），
    // 注意这里是指针，后面slideWindow中会赋新值，这里只是提前占座（知乎上有人问：
    // https://www.zhihu.com/question/63754583/answer/259699612
    if (marginalization_flag == MARGIN_OLD)  // marg 最老帧
    {
        /**
         *  3.1 对于边缘化首帧
         *     3.1.1 把之前存的残差部分加进来；
         *     3.1.2 把与首帧相关的残差项加进来，包含IMU、vision；
         *     3.1.3 计算所有残差项的残差和雅克比；
         *     3.1.4 多线程构造 Hx=b 的结构（需要细看）；
         *     3.1.5 marg结束，调整参数块在下一次window的位置；
         */
        // 向ResidualBlockInfo容器中(factors)添加先验残差、最新IMU测量残差、camera所有特征点测量残差
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        
        vector2double();

        //! 先验误差会一直保存，而不是只使用一次
        //! 如果上一次边缘化的信息存在，要边缘化的参数块是 para_Pose[0] para_SpeedBias[0]
        //      以及 para_Feature[feature_index](滑窗内的第feature_index个点的逆深度)
        // 1. 把上一次先验项中的残差项(尺寸为 n)传递给当前先验项，并从中去除需要丢弃的状态量
        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            
            //! 构造边缘化的的Factor
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            
            //! 添加上一次边缘化的参数块
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        
        // 2. 将滑窗内第0帧和第1帧间的IMU预积分因子(pre_integrations[1])放到 marginalization_info 中，
        //    即图中上半部分中x0和x1之间的表示IMU约束的黄色块
        //！添加IMU的先验，只包含边缘化帧的IMU测量残差
        //！Question：不应该是pre_integrations[0]么
        //!
        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                // 需要Marg的是最老帧，所以是第0帧与第1帧之间的pre_integrations
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                // dropset{0, 1} 对应marg帧的Pose、SpeedBias
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 3. 挑选出第一次观测帧为第0帧的路标点，将对应的多组视觉观测放到 marginalization_info 中，
        //    即图中上半部分中x0所看到的红色五角星的路标点
        //！添加视觉的先验，只添加起始帧是旧帧且观测次数大于2的Features
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                
                // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
//                if (!it_per_id.isPriorFeature())
                if (it_per_id.used_num < 4)  // TODO: 不一致
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                
                //! 只选择被边缘化的帧的Features
                // 找在第0帧观测到的特征点
                // TODO: 移动滑动窗口后，frameid是否会被更新，从0开始重新计数？
                if (imu_i != 0)
                    continue;

                //! 得到该Feature在起始观测帧下的归一化坐标
                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    //! 和起始观测帧是同一帧
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }
        
        //! 将三个ResidualBlockInfo中的参数块综合到marginalization_info中
        //  计算所有ResidualBlock(残差项)的残差和雅克比,parameter_block_data是参数块的容器
        
        TicToc t_pre_margin;
        // 根据各个测量模型Evaluate() 计算残差；
        // 各个参数块拷贝到统一的内存（parameter_block_data）中
        // 4. 得到IMU和视觉观测(cost_function)对应的参数块(parameter_blocks)，雅可比矩阵，残差值(residuals)
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        // 5. 多线程计整个先验项的参数块，雅可比矩阵和残差值，对应舒尔补公式（9-7）
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        /**
         * 6. 最后移交了优化项需要得到的两个变量：last_marginalization_info和last_marginalization_parameter_blocks
         */
        //！将滑窗里关键帧位姿移位，为什么是向右移位了呢？
        //! 这里是保存了所有状态量的信息，为什么没有保存逆深度的状态量呢
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            // TODO: what's the meaning
            // 可以把一个指针转换成一个整数，也可以把一个整数转换成一个指针
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else  // marginalization_flag == MARGIN_SECOND_NEW  marg次新帧
    {
        /**
         *  3.2 对于边缘化倒数第二帧
         *      3.2.1 如果倒数第二帧不是关键帧，保留该帧的IMU测量，去掉该帧的visual；代码中都没写；
         *      3.2.2 计算所有残差项的残差和雅克比；
         *      3.2.3 多线程构造 Hx=b 的结构（需要细看）；
         *      3.2.4 marg结束，调整参数块在下一次window的位置。
         */
        //！边缘化倒数第二帧，如果倒数第二帧不是关键帧
        // 1.保留该帧的IMU测量,去掉该帧的visual,代码中都没有写.
        // 2.premarg
        // 3.marg
        // 4.滑动窗口移动
        
        // 如果倒数第二帧不是关键帧，则把这帧的视觉测量舍弃掉（边缘化）但保留IMU测量值在滑动窗口中。（其它步骤和上一步骤相同）
        // 1.保留该帧的IMU测量，margin该帧的visual
        // 2.premargin
        // 3.marginalize
        // 4.滑动窗口移动（去掉倒数第二个）
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    //！寻找导数第二帧的位姿
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

/**
 * marginalize the state from the sliding window and change feature start frame
 * 滑动窗口all_image_frame，维持滑动窗口的大小，保证SLAM运行计算的复杂度。
 * 如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，
 * 时间最长的一帧和其测量值就会被边缘化掉；如果第二最新帧不是关键帧的话，
 * 则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中，这样的策略会保证系统的稀疏性。
 * (0, 1, …, N)关键帧，0是时间最长的关键帧，N是最新关键帧。
 */
// 实际滑动窗口的地方，如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，
// 时间最长的一帧和其测量值就会被边缘化掉如果第二最新帧不是关键帧的话，则把这帧的
// 视觉测量舍弃掉而保留IMU测量值在滑动窗口中这样的策略会保证系统的稀疏性
void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);      // 变换矩阵
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            
            // 删除一帧
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            // 释放ImageFrame中最老帧信息
            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();  // 删除最老帧更新窗口
        }
    }
    else  // non keyframe (marginalization_flag == MARGIN_SECOND_NEW)
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();  // 删除次新帧信息
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

/**
 * FeatureManager::removeBack() 将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
 */
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;

        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack(); // 将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    
    featureTracker.setPrediction(predictPts);
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

/**
 * 调用了fastPredictIMU()，用来预测最新P,V,Q的姿态 latest_*最新时刻的姿态。
 * 这个的作用是为了刷新姿态的输出，但是这个值的误差相对会比较大，是未经过非线性优化获取的初始值。
 */
void Estimator::updateLatestStates()
{
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d> > tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d> > tmp_gyrBuf = gyrBuf;
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mBuf.unlock();
}


