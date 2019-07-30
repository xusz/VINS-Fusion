/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
// 完全一致
#pragma once

#include "utility.h"
#include <ceres/ceres.h>
#include "global_param.h"

using namespace Eigen;

class IntegrationBase
{
public:
    IntegrationBase() = delete;
    
    // 调用imu的预积分，计算对应的状态量、协方差和雅可比矩阵
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
            jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0)   =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3)   =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6)   =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9)   =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }
    
    // push_back 中进行预积分处理
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        propagate(dt, acc, gyr);  // 积分
    }
    
    // 优化过程中Bias会更新，有时候需要根据新的bias重新计算预积分
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        
        jacobian.setIdentity();
        covariance.setZero();
        
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }
    
    /**
     * 使用中点积分方法midPointIntegration计算预积分的测量值，
     * 中点积分法中主要包含两个部分，分别是得到状态变化量result_delta_q，
     * result_delta_p，result_delta_v，result_linearized_ba，
     * result_linearized_bg和得到跟新协方差矩阵和雅可比矩阵
     *（注意，虽然得到了雅各比矩阵和协方差矩阵，但是还没有求残差和修正偏置一阶项的状态变量），
     * 由于使用的是中点积分，所以需要上一个时刻的IMU数据，
     * 包括测量值加速度和角速度以及状态变化量，初始值由构造函数提供
     *
     * 每一次处理 Jacobian都会更新，但是请注意这个【Jacobian】并不是IMU残差的【Jacobian】
     *
     * 计算状态转移矩阵F，雅克比矩阵 jacobian = (F+I)*jacobian，和协方差矩阵
     */
    // 中值积分法: https://www.zhihu.com/question/64381223/answer/255818747
    /**
     * 求状态向量对bias的Jacobian，当bias变化较小时，使用Jacobian去更新状态；
     * 否则需要以当前imu为参考系，重新预积分，对应repropagation()。
     * 同时，需要计算error state model中误差传播方程的系数矩阵F和V
     */
    /**
     * pre-integration
     * time interval of two imu; last and current imu measurements;
     * p,q,v relate to local frame; ba and bg; propagated p,q,v,ba,bg;
     * whether to update Jacobian and calculate F,V
     */
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        // ROS_INFO("midpoint integration");
        // mid-point integration with bias = 0
        
        /**
         * 两帧之间PVQ增量的中值离散形式(IMU坐标系)，对应5.7节
         * 直接使用IMU坐标系积分，减少坐标系间的转换
         */
        
        // 对应5.7节中的公式（5-30）的前半部分
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        // 对应5.7节中的公式（5-31）
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        
        // 对应5.7节中的公式（5-29）
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);

        // 对应5.7节中的公式（5-30）的后半部分
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        // 对应5.7节中的公式（5-30）
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        
        // 对应5.7节中的公式（5-27）
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        // 对应5.7节中的公式（5-28）
        result_delta_v = delta_v + un_acc * _dt;
        
        // ba and bg donot change
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
        
        // 这个jacobian 和IMU_factor jacobian 有什么不同???
        // 这个jacobian 是对bias的jacobian
        // IMU_factor_jacobian 是对状态的jacobian
        // jacobian to bias, used when the bias changes slightly and no need of repropagation
        // 当偏差稍微改变而不需要重新传播时使用
        if (update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;
            
            R_w_x << 0,      -w_x(2),   w_x(1),
                     w_x(2),  0,       -w_x(0),
                    -w_x(1),  w_x(0),   0;
            
            R_a_0_x << 0,        -a_0_x(2),  a_0_x(1),
                       a_0_x(2),  0,        -a_0_x(0),
                      -a_0_x(1),  a_0_x(0),  0;
            
            R_a_1_x << 0,        -a_1_x(2),  a_1_x(1),
                       a_1_x(2),  0,        -a_1_x(0),
                      -a_1_x(1),  a_1_x(0),  0;
            
            // error state model
            // should use discrete format and mid-point approximation
            MatrixXd F = MatrixXd::Zero(15, 15);
            // F_00
            F.block<3, 3>(0, 0)   =  Matrix3d::Identity();
            // F_01
            F.block<3, 3>(0, 3)   =  -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
            -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            // F_02
            F.block<3, 3>(0, 6)   =  MatrixXd::Identity(3,3) * _dt;
            // F_03
            F.block<3, 3>(0, 9)   =  -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            // F_04
            F.block<3, 3>(0, 12)  =  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            // F_11
            F.block<3, 3>(3, 3)   =  Matrix3d::Identity() - R_w_x * _dt;
            // F_14
            F.block<3, 3>(3, 12)  =  -1.0 * MatrixXd::Identity(3,3) * _dt;
            // F_21
            F.block<3, 3>(6, 3)   =  -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
            -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            // F_33
            F.block<3, 3>(6, 6)   =  Matrix3d::Identity();
            // F_23
            F.block<3, 3>(6, 9)   =  -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            // F_24
            F.block<3, 3>(6, 12)  =  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            // F_33
            F.block<3, 3>(9, 9)   =  Matrix3d::Identity();
            // F_44
            F.block<3, 3>(12, 12) =  Matrix3d::Identity();
            // cout<<"A"<<endl<<A<<endl;
            
            MatrixXd V = MatrixXd::Zero(15, 18);
            // V_00
            V.block<3, 3>(0, 0)   =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            // V_01
            V.block<3, 3>(0, 3)   =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
            // V_02
            V.block<3, 3>(0, 6)   =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            // V_03
            V.block<3, 3>(0, 9)   =  V.block<3, 3>(0, 3);
            // V_11
            V.block<3, 3>(3, 3)   =  0.5 * MatrixXd::Identity(3,3) * _dt;
            // V_13
            V.block<3, 3>(3, 9)   =  0.5 * MatrixXd::Identity(3,3) * _dt;
            // V_20
            V.block<3, 3>(6, 0)   =  0.5 * delta_q.toRotationMatrix() * _dt;
            // V_21
            V.block<3, 3>(6, 3)   =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
            // V_22
            V.block<3, 3>(6, 6)   =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            // V_23
            V.block<3, 3>(6, 9)   =  V.block<3, 3>(6, 3);
            // V_34
            V.block<3, 3>(9, 12)  =  MatrixXd::Identity(3,3) * _dt;
            // V_45
            V.block<3, 3>(12, 15) =  MatrixXd::Identity(3,3) * _dt;
            
            // step_jacobian = F;
            // step_V = V;
            
            // 当前误差状态量关于预积分初始时刻误差状态量的雅可比矩阵
            jacobian = F * jacobian;  // 对bias的jacobian
            // 误差状态的协方差矩阵，带有 noise
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }
    }
    
    /**
     * 预积分传播方程，使用中点积分方法
     */
    // 积分计算两个关键帧之间IMU测量的变化量：旋转delta_q 速度delta_v 位移delta_p，
    // 加速度的biaslinearized_ba 陀螺仪的Biaslinearized_bg
    // 同时维护更新预积分的Jacobian和Covariance，计算优化时必要的参数
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
        
        /*
         * 使用中点积分方法midPointIntegration计算预积分的测量值，
         * 中点积分法中主要包含两个部分，分别是得到状态变化量result_delta_q，
         * result_delta_p，result_delta_v，result_linearized_ba，
         * result_linearized_bg和得到跟新协方差矩阵和雅可比矩阵
         *（注意，虽然得到了雅各比矩阵和协方差矩阵，但是还没有求残差和修正偏置一阶项的状态变量），
         * 由于使用的是中点积分，所以需要上一个时刻的IMU数据，
         * 包括测量值加速度和角速度以及状态变化量，初始值由构造函数提供
         *
         * 注：每一次的处理，Jacobian都会更新，但是请注意这个Jacobian并不是IMU残差的Jacobian
         */
        // 中点积分
        // 输入:
        // _dt:当前IMU数据和前一个IMU数据的时间差
        // acc_0:前一个IMU加速度计数据
        // gyro_0:前一个IMU陀螺仪数据
        // _acc_1:当前IMU加速度计数据
        // _gyr_1:当前IMU加速度计数据
        // delta_p, delta_q, delta_v:前一个IMU预计分测量值
        // linearized_ba, linearized_bg:前一个ba和bg
        // 输出:
        // result_delta_p, result_delta_q, result_delta_v:当前IMU预计分测量值
        // result_linearized_ba, result_linearized_bg:当前ba和bg
        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);
        
        // checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1,
        //               delta_p, delta_q, delta_v,
        //               linearized_ba, linearized_bg);
        
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();  // 归一化
        
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
    }
    
    /**
     * evaluate函数在这个函数里面进行了状态变化量的偏置一阶修正以及残差的计算
     */
    // IMU 残差，对应 8.3.1 节
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;
        
        // 对应IMU预积分中，计算出的大雅可比矩阵 jacobian
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);
        
        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);
        
        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);
        
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        
        /**
         * IMU测量量 Δα Δβ Δγ (corrected_delta_*) 的一阶表示，对应5.12节中的公式（Y-12）
         */
        // IMU預積分的結果，消除掉acc bias和gyro bias的影響, 對應IMU model中的\hat{\alpha},\hat{\beta},\hat{\gamma}
        // 公式(6) 一阶估计值：γ
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        // 公式(6) 一阶估计值：β^{b_k}_{b_{k+1}} = \hat{β}^{b_k}_{b_{k+1}} + J_a + J_w
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        // 公式(6) 一阶估计值：α^{b_k}_{b_{k+1}} = \hat{α}^{b_k}_{b_{k+1}} + J_a + J_w
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        
        // IMU項residual計算，輸入參數是狀態的估計值，上面corrected_delta_* 是預積分值, 二者求'diff'得到residual.
        // corrected_delta_* 是经过校正过bias的(根据p,q,v对bias的jacobian以及bias的差对预积分量进行修正),只有acc和gyro噪声,是跟bias相关的量, 而跟初始时刻的速度及姿态都无关。
        // 在优化迭代的过程中, 预积分值是不变的, 输入的状态值会被不断的更新, 然后不断的调用evaluate()计算更新后的IMU残差
        
        // 残差对应8.3.1节中的公式（8-10）或者 8.3.4节中的公式（8-17）
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        
        return residuals;
    }


public:
    /*
     * 中值积分需要使用前后两个时刻的IMU数据
     */
    double dt;   // 前后两个IMU的时间间隔
    Eigen::Vector3d acc_0, gyr_0;   // 前一帧IMU数据中的加速度计测量值和陀螺仪测量值
    Eigen::Vector3d acc_1, gyr_1;   // 后一帧IMU数据中的加速度计测量值和陀螺仪测量值
    
    // 这一段预积分初始时刻的IMU测量值，作为常量一直保存，在IntegrationBase对象创建时指定
    const Eigen::Vector3d linearized_acc, linearized_gyr;
    // 这一段预积分对应的加速度计偏置和陀螺仪偏置
    Eigen::Vector3d linearized_ba, linearized_bg;
    
    // jacobian: 当前误差状态量关于预积分初始时刻误差状态量的雅可比矩阵
    // covariance: 误差状态的协方差矩阵
    // jacobian to bias, used when the bias changes slightly and no need of repropagation
    Eigen::Matrix<double, 15, 15> jacobian;      // 对bias的雅可比矩阵
    Eigen::Matrix<double, 15, 15> covariance;    // 协方差矩阵  LLT时会用到
    Eigen::Matrix<double, 15, 15> step_jacobian; // 似乎是用来调试程序的临时变量
    Eigen::Matrix<double, 15, 18> step_V;        // 似乎是用来调试程序的临时变量
    Eigen::Matrix<double, 18, 18> noise;         // 误差状态传播方程中的噪声的协方差矩阵
    
    double sum_dt;   // 一段预积分的总时间间隔
    // delta_p 表示该段预积分初始时刻本体坐标系下，当前时刻本体坐标系的位置
    Eigen::Vector3d delta_p;     // 平移预积分结果(每次有IMU输入就计算，是累积值)
    // delta_q 表示该段预积分初始时刻本体坐标系下，当前时刻本体坐标系的旋转
    Eigen::Quaterniond delta_q;  // 旋转预积分结果(每次有IMU输入就计算，是累积值)
    // delta_v 表示该段预积分初始时刻本体坐标系下，当前时刻本体坐标系的速度
    Eigen::Vector3d delta_v;     // 速度预积分结果(每次有IMU输入就计算，是累积值)
    
    /*
     * 该段预积分所使用的IMU数据的缓存vector，每一帧Frame会对应多个IMU data
     * 这3个缓存的作用是：当bias变换过大时，需要使用这些数据重新进行预积分
     */
    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
};

