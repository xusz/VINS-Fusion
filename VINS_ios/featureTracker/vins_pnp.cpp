//
//  vins_pnp.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "vins_pnp.hpp"

/**
 * （猜测）可能是为了 drawAR 使用，需要及时计算PVQ
 */

vinsPnP::vinsPnP()
{
    printf("init VINS_pnp begins\n");
    clearState();
}

void vinsPnP::setIMUModel()
{
    PerspectiveFactor::sqrt_info = FOCAL_LENGTH_X / 1.5 * Matrix2d::Identity();
}

void vinsPnP::clearState()
{
    printf("clear state\n");
    
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        pre_integrations[i] = nullptr;
        
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
        features[i].clear();
        find_solved[i] = false;
    }
    
    tic << TIC_X, TIC_Y, TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y, RIC_p, RIC_r));
    
    frame_count = 0;
    first_imu = false;
}

void vinsPnP::setExtrinsic()
{
    tic << TIC_X, TIC_Y, TIC_Z;
    ric = Utility::ypr2R(Vector3d(RIC_y, RIC_p, RIC_r));
    printf("pnp set extrinsic %lf %lf %lf\n", tic.x(), tic.y(), tic.z());
}

/**
 * 设置小滑窗内第i帧的PVQ
 * 对PnP的小滑窗（比如6个相邻帧）内的PVQB进行赋值，当小滑窗内有第i帧时，则用从后端传过来的第i帧PVQ来更新
 */
void vinsPnP::setInit(VINS_RESULT vins_result)
{
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Bas[i] = vins_result.Ba;
        Bgs[i] = vins_result.Bg;
        
        if (Headers[i] == vins_result.header)
        {
            find_solved[i] = true;
            find_solved_vins[i].P = vins_result.P;
            find_solved_vins[i].R = vins_result.R;
            find_solved_vins[i].V = vins_result.V;
            Ps[i] = vins_result.P;
            Rs[i] = vins_result.R;
            Vs[i] = vins_result.V;
            
            printf("pnp find new index %d\n", i);
        }
    }
    
    //printf("pnp vins index %d header_vins %lf, header_pnp %lf \n", solved_index, vins_result.header, Headers[solved_index]);
    //printf("pnp init value %lf %lf %lf\n",Ps[solved_index].x(), Ps[solved_index].y(), Ps[solved_index].z());
}

/**
 * 更新小滑窗中与第k帧有共视关系的帧的3D点坐标P_w
 */
void vinsPnP::updateFeatures(vector<IMG_MSG_LOCAL> &feature_msg)
{
    for (int i = 0; i < frame_count; i++)
    {
        int j = 0;
        for (auto &it : feature_msg)
        {
            while (features[i][j].id < it.id)
            {
                j++;
            }
            // 更新feature 在最新帧中的位置
            if (features[i][j].id == it.id)
            {
                // TODO: position 需要更新？？？
                features[i][j].position = it.position;
                features[i][j].track_num = it.track_num;
                
                // TODO: 是否可以 break;
            }
        }
    }
}

/**
 * 数据类型转换
 * Ps,Rs转换为para_Pose
 * Vs,Bas,Bgs转换为para_SpeedBias
 * tic,ric转换为para_Ex_Pose
 */
void vinsPnP::old2new()
{
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        
        para_Speed[i][0] = Vs[i].x();
        para_Speed[i][1] = Vs[i].y();
        para_Speed[i][2] = Vs[i].z();
        
        para_Bias[i][0] = Bas[i].x();
        para_Bias[i][1] = Bas[i].y();
        para_Bias[i][2] = Bas[i].z();
        
        para_Bias[i][3] = Bgs[i].x();
        para_Bias[i][4] = Bgs[i].y();
        para_Bias[i][5] = Bgs[i].z();
        
        // printf("pnp before solve header %lf %d P: %lf %lf %lf Bgs: %lf %lf %lf\n",
        //      Headers[i], find_solved[i], Bas[i].x(), Bas[i].y(), Bas[i].z(), Bgs[i].x(), Bgs[i].y(), Bgs[i].z());
        // printf("pnp before solve header %lf %d P: %lf %lf %lf\n",Headers[i],
        //      find_solved[i], Ps[i].x(), Ps[i].y(), Ps[i].z());
    }
    
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
}

void vinsPnP::new2old()
{
    int solved_index = 0;
    for (int i = 0; i < PNP_SIZE + 1; i++)
    {
        if (find_solved[i])
        {
            solved_index = i;
            printf("pnp find index %d\n", i);
            break;
        }
    }
    
    solved_index = 0;
//    Vector3d origin_R0 = Utility::R2ypr(Rs[solved_index]);
    
    Vector3d origin_P0 = Ps[solved_index];
    
//    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[solved_index][6],
//                                                     para_Pose[solved_index][3],
//                                                     para_Pose[solved_index][4],
//                                                     para_Pose[solved_index][5]).toRotationMatrix());
    
    // TODO:
//    double y_diff = origin_R0.x() - origin_R00.x();
//    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        /*
         Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
         Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[solved_index][0],
         para_Pose[i][1] - para_Pose[solved_index][1],
         para_Pose[i][2] - para_Pose[solved_index][2]) + origin_P0;
         Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
         para_SpeedBias[i][1],
         para_SpeedBias[i][2]);
         */
        Rs[i] = Quaterniond(para_Pose[i][6],
                            para_Pose[i][3],
                            para_Pose[i][4],
                            para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        
        Vs[i] = Vector3d(para_Speed[i][0], para_Speed[i][1], para_Speed[i][2]);
        
        Bas[i] = Vector3d(para_Bias[i][0],
                          para_Bias[i][1],
                          para_Bias[i][2]);
        
        Bgs[i] = Vector3d(para_Bias[i][3],
                          para_Bias[i][4],
                          para_Bias[i][5]);
        
        //printf("pnp after solve header %lf %d P: %lf %lf %lf Bgs: %lf %lf %lf\n",Headers[i], find_solved[i], Bas[i].x(), Bas[i].y(), Bas[i].z(), Bgs[i].x(), Bgs[i].y(), Bgs[i].z());
        
//        Vector3d R_ypr = Utility::R2ypr(Rs[i]);
        
        //printf("pnp after solve header %lf %d P: %lf %lf %lf R: %lf %lf %lf\n",Headers[i], find_solved[i], Ps[i].x(), Ps[i].y(), Ps[i].z(), R_ypr.x(), R_ypr.y(), R_ypr.z());
        //printf("pnp after solve header %lf %d\n",Headers[i], find_solved[i]);
    }
}

/**
 * 对第k-1帧到第k帧预积分
 */
void vinsPnP::processIMU(double dt, const Vector3d &linear_acceleration,
                         const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0,
                                                            Bas[frame_count],
                                                            Bgs[frame_count]};
    }
    
    if (frame_count != 0)
    {
        // covariance propagate
        // 每次pre_integration push一个IMU数据后就会进行积分： IntegrationBase::push_back
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        // 当前时刻PVQ，midpoint integration
        {
            Vector3d g{0, 0, GRAVITY};
            int j = frame_count;
            
            // 下面都采用的是中值积分的传播方式， noise是zero mean Gassu，在这里忽略了
            
            // 对应5.3节中的公式（5-13）的前半部分
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            // 对应5.3节中的公式（5-14）
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
           
            // 各帧在世界坐标系中的角度，对应5.3节中的公式（5-12）
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
    }
    
    acc_0 = linear_acceleration;  // 线加速度
    gyr_0 = angular_velocity;     // 陀螺仪/角加速度
}

/**
 * 进行小滑窗优化
 * 首先通过updateFeatures更新小滑窗中与第k帧有共视关系的帧的3D点坐标P_w，
 * 最后调用solve_ceres来优化小滑窗内的PVQ，不优化路标点，
 * 涉及的两个约束为：IMU的帧间约束，和每一帧的PnP（即每一帧的2D-3D观测）的视觉重投影误差约束
 */
void vinsPnP::processImage(vector<IMG_MSG_LOCAL> &feature_msg,
                           double header, bool use_pnp)
{
    printf("pnp %d adding feature points %lu \n",
        frame_count, feature_msg.size());
    
    //add feature
    features[frame_count] = feature_msg;
    Headers[frame_count] = header;
    
    // 更新滑窗内的路标点位置Pw（TODO: what's mean）
    updateFeatures(feature_msg);
    
    if (frame_count < PNP_SIZE)
    {
        frame_count++;
        return;
    }
    
    // 优化小滑窗内的PVQ，不优化路标点
    // 涉及的两个约束为：IMU的帧间约束，和每一帧的PnP（即每一帧的2D-3D观测）的视觉重投影误差约束
    if (use_pnp)
    {
        solve_ceres();
    }
    
    // 进行滑窗优化
    slideWindow();
}

/**
 * 优化小滑窗内的PVQ，不优化路标点，
 * 涉及的两个约束为：IMU的帧间约束，和每一帧的PnP（即每一帧的2D-3D观测）的视觉重投影误差约束
 */
void vinsPnP::solve_ceres()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    
    loss_function = new ceres::CauchyLoss(1.0);
    int start_index = 0;
    /*
     for(int i = 0; i < PNP_SIZE + 1; i++)
     {
     if(find_solved[i])
     {
     start_index = i;
     break;
     }
     }
     */
    
    /*
     * 添加ceres参数块，包括位姿，速度，零偏，外参，时间偏置
     */
    
    for (int i = start_index; i < PNP_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_Speed[i], SIZE_SPEED);
        problem.AddParameterBlock(para_Bias[i], SIZE_BIAS);
        
        if (find_solved[i])
        {
            problem.SetParameterBlockConstant(para_Pose[i]);
            problem.SetParameterBlockConstant(para_Speed[i]);
        }
        problem.SetParameterBlockConstant(para_Bias[i]);
    }
    
    for (int i = start_index; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i],
                                  SIZE_POSE,
                                  local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    old2new();
    
    // IMU factor, 添加IMU残差
    for (int i = start_index; i < PNP_SIZE; i++)
    {
        int j = i + 1;
        IMUFactorPnP* imu_factor = new IMUFactorPnP(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL,
                                 para_Pose[i],
                                 para_Speed[i],
                                 para_Bias[i],
                                 para_Pose[j],
                                 para_Speed[j],
                                 para_Bias[j]);
    }
    
    for (int i = start_index; i <= PNP_SIZE; i++)
    {
        for (auto &it : features[i])
        {
            PerspectiveFactor *f = new PerspectiveFactor(it.observation,
                                                         it.position,
                                                         it.track_num);
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[i],
                                     para_Ex_Pose[0]);
        }
    }
    
    ceres::Solver::Options options;
    // 使用舒尔补
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 5; // # max solver itrations, to guarantee real time
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = 0.01;
    
    ceres::Solver::Summary summary;
    TS(ceres_pnp);
    printf("solve pnp\n");
    ceres::Solve(options, &problem, &summary);
    TE(ceres_pnp);
    
    new2old();
}

/*
 * marginalize the state from the sliding window and change feature start frame
 */
void vinsPnP::slideWindow()
{
    if (frame_count == PNP_SIZE)
    {
        for (int i = 0; i < PNP_SIZE; i++)
        {
            Rs[i].swap(Rs[i + 1]);
            std::swap(pre_integrations[i], pre_integrations[i + 1]);
            dt_buf[i].swap(dt_buf[i + 1]);
            linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
            angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
            Headers[i] = Headers[i + 1];
            Ps[i].swap(Ps[i + 1]);
            Vs[i].swap(Vs[i + 1]);
            std::swap(features[i], features[i + 1]);
            find_solved[i] = find_solved[i + 1];
            std::swap(find_solved_vins[i], find_solved_vins[i + 1]);
        }
        
        Headers[PNP_SIZE] = Headers[PNP_SIZE - 1];
        Ps[PNP_SIZE] = Ps[PNP_SIZE - 1];
        Vs[PNP_SIZE] = Vs[PNP_SIZE - 1];
        Rs[PNP_SIZE] = Rs[PNP_SIZE - 1];
        Bas[PNP_SIZE] = Bas[PNP_SIZE - 1];
        Bgs[PNP_SIZE] = Bgs[PNP_SIZE - 1];
        find_solved[PNP_SIZE] = false;
        
        if (pre_integrations[PNP_SIZE] != NULL)
        {
            delete pre_integrations[PNP_SIZE];
        }
        
        pre_integrations[PNP_SIZE] = new IntegrationBase{acc_0, gyr_0,
                                                        Bas[PNP_SIZE],
                                                        Bgs[PNP_SIZE]};
        features[PNP_SIZE].clear();
        dt_buf[PNP_SIZE].clear();
        linear_acceleration_buf[PNP_SIZE].clear();
        angular_velocity_buf[PNP_SIZE].clear();
    }
}

