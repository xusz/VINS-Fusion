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

#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;

// 全局 smf Feature
struct SFMFeature
{
    bool state;  // 该特征点是否被三角化
    int id;
    vector<pair<int, Vector2d>> observation;  // 观测到该特征点的帧号及帧坐标系？
    double position[3];  // 三角化后的坐标位置（world坐标系？）
    double depth;
};

struct ReprojectionError3D
{
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u(observed_u), observed_v(observed_v)
		{}

	template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
	{
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
		T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        return true;
    }
    
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y)
    {
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_x,observed_y)));
	}

    double observed_u;
    double observed_v;
};

class GlobalSFM
{
public:
    GlobalSFM();
    
    /*
     * \brief 外部調用接口,主要處理函數. 輸入第l幀和最後一幀的相對R,t,
     *        根據特徵點的觀測估計所有幀的位姿和特徵點的3D座標
     \param[in] frame_num: pose的個數, elements in q,T
     \param[out] q: SFM結果,每幀在l幀參考系下的quaternion
     \param[out] T: SFM結果,每幀在l幀參考系下的position
     \param[in] l: 以第l幀爲參考系,即l幀的pose爲座標原點
     \param[in] relative_R: 第l幀到最後一幀的相對旋轉
     \param[in] relative_T: 第l幀到最後一幀的相對平移
     \param[in] sfm_f: feature list,每個SFMFeature中包含多個觀測
     \param[out] sfm_tracked_point: 優化後的3D特徵點在l幀參考系的position
     */
	bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

private:
    /**\brief 根據當前已經三角化的特徵點估計某一幀的R,t */
    bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);


    
    /**\brief 輸入兩個pose和2D觀測點, 三角化3D特徵點 */
	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
							Vector2d &point0, Vector2d &point1, Vector3d &point_3d);
    
    /** 輸入兩幀的pose, 三角化它們共同觀測的特徵點, 之前已經被三角化的特徵點不再處理 */
    void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                              vector<SFMFeature> &sfm_f);
    
    int feature_num;
};


