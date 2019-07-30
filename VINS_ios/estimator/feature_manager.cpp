/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
		
        // used number can be:[1], 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        // start frame can be:0, 1, 2, 3, 4, 5, 6, 7, [8, 9, 10]
//        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        if (it.used_num >= 4)  // TODO: 不一致
        {
            cnt++;
        }
    }
    
    return cnt;
}

/*
 Check if the current frame has enough parallax compare with previous frame
 if have, return true;
 if no, return false;
 At the sametime, add the new feature observation to the feature class
 */
/**
 * 通过检测两帧之间的视差决定是否作为关键帧，
 * 同时添加之前检测到的特征点到feature（list< FeaturePerId >）容器中，
 * 计算每一个点跟踪的次数，以及它的视差
 */
/**
 * @brief 把图像特征点放入名为feature的list容器中，然后计算当前的视差
 */
/** 选KF策略
 * 把当前帧图像（frame_count）的特征点添加到feature容器中
 * 计算第2最新帧与第3最新帧之间的平均视差（当前帧是第1最新帧）
 * 也就是说当前帧图像特征点存入feature中后，并不会立即判断是否将当前帧添加为新的关键帧，而是去判断当前帧的前一帧（第2最新帧）。
 * 当前帧图像要在下一次接收到图像时进行判断（那个时候，当前帧已经变成了第2最新帧）
 */
// 向Featuresmanger中添加Features并确定共视关系及视差角的大小
/**
 * 1 FeaturePerFrame 特征点在某个图像帧下的坐标（归一化平面坐标）
 * 2 遍历特征点,看该特征点是否在特征点的列表中,如果没在,则将<FeatureID,Start_frame>存入到Feature列表中(如果该Feature之前被观测到过,统计数目)
 * 3 计算共视关系， parallax_num为满足要求的Feature的个数
 *      3.1 至少有两帧观测到该特征点(起始帧要小于倒数第二帧，终止帧要大于倒数第二帧，这样至少有三帧观测到该Feature(包括当前帧))
 *      3.2 判断观测到该特征点的frame中倒数第二帧和倒数第三帧的共视关系 实际是求取该特征点在两帧的归一化平面上的坐标点的距离ans
 *      3.3 得到视差总和并统计个数
 * 4 得到平均视差,平均视差视差要大于某个阈值, MIN_PARALLAX=10，大约是10个像素点.
 *      这样的帧即是关键帧,边缘化最老的帧.否则不是关键帧,边缘化第二最新帧（次新帧）
 *
 * image_msg -- point_clouds，当前帧的点云数据
 */
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;  // 第2最新帧和第3最新帧之间跟踪到的特征点的总视差
    int parallax_num = 0;     // 第2最新帧和第3最新帧之间跟踪到的特征点的数量(满足某些条件的特征点个数)
    last_track_num = 0;       // 当前帧（第1最新帧）图像跟踪到的特征点的数量
    double last_average_parallax = 0;
    int new_feature_num = 0;
    int long_track_num = 0;   // 某个特征点至少被4个关键帧观测到，认为是 long_track
    
    // 每个feature有可能出现多个帧中，share same id，放入feature容器中
    // feature容器按照特征点id组织特征点数据，对于每个id的特征点，
    // 记录它被滑动窗口中哪些图像帧观测到了
    for (auto &id_pts : image)
    {
        // 特征点管理器，存储特征点格式：首先按照特征点ID，一个一个存储，每个ID会包含其在不同帧上的位置
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
                              return it.feature_id == feature_id;
                          });
        // new feature
        // 返回尾部迭代器，说明该特征点第一次出现（在当前帧中新检测的特征点），
        // 需要在feature中新建一个FeaturePerId对象
        if (it == feature.end()) // 如果没有找到此ID，就在管理器中增加此特征点
        {
            // give id and start frame index
            feature.push_back(FeaturePerId(feature_id, frame_count));
            // give point
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id) // find match with previous feature
        {
            // 如果找到了相同ID特征点，就在其FeaturePerFrame内增加此特征点在此帧的位置以及其他信息，
            // 然后增加last_track_num，说明此帧有多少个相同特征点被跟踪到
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;   // 当前帧（最新帧）图像跟踪到的特征点的数量
            if (it->feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }
    
    // 1. 当前帧的帧号小于2，即为0或1，为0，则没有第2最新帧，为1，则第2最新帧是滑动窗口中的第1帧
    // 2. 当前帧（第1最新帧）跟踪到的特征点数量小于20（？？？为什么当前帧的跟踪质量不好，就把第2最新帧当作关键帧？？？）
    // 出现以上2种情况的任意一种，则认为第2最新帧是关键帧
    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    /* 关键帧数小于2 跟踪到角点数目小于20 连续被跟踪小于40帧 当前帧加入角点记录列表的数目大于跟踪到角点数目的一半(当前帧出现新的视野较多)
     * 以上4条件满足一则插入滑动窗口关键帧
     */
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;
    
    // 计算视差，second last和third last
    // 计算第2最新帧和第3最新帧之间跟踪到的特征点的平均视差
    for (auto &it_per_id : feature)  // 滑动窗口中所有特征点
    {
        // 计算能被当前帧和其前两帧共同看到的特征点视差
        // 从特征点被观测到的起始帧->最后一帧，持续被观测到
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            // 对相近的特征点进行视差计算
            // 对于给定id的特征点，计算第2最新帧和第3最新帧之间该特征点的视差（当前帧frame_count是第1最新帧）
            //（需要使用IMU数据补偿因旋转造成的视差）
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0) // 插入关键帧
    {
        // 如果第2最新帧和第3最新帧之间跟踪到的特征点的数量为0，则把第2最新帧添加为关键帧
        // ？？怎么会出现这种情况？？？？
        // 如果出现这种情况，那么第2最新帧和第3最新帧之间的视觉约束关系不就没有了？？？
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // 如果平均视差大于设定的阈值，则把第2最新帧当作关键帧
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

/**
 * 得到两帧之间特征点共视关系
 * 第[l,r]帧中都包含某Feature
 */
vector<pair<Vector3d, Vector3d> > FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d> > corres;
    for (auto &it : feature)
    {
        // [l,r]帧中都包含此feature
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;
            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
//        if (!it_per_id.isPriorFeature())
        if (it_per_id.used_num < 4) // TODO: 不一致
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);

        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = SOLVE_FAIL;
        }
        else
            it_per_id.solve_flag = SOLVE_SUCC;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == SOLVE_FAIL)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
//        if (!it_per_id.isPriorFeature())
        if (it_per_id.used_num < 4) // TODO: 不一致
            continue;

#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

/**
 * 计算函数提供了cv::solvePnP函数的外层封装和数据准备，
 * 在初始化第1帧图像时，因深度信息匮乏先执行三角化求深度。
 */
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();        // 相机到世界 R
    P_initial = -(R_initial * P);   // 相机到世界 T

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)     // 最小二乘法至少4个角点
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);    // 罗德里格斯变换，向量变换到矩阵
    cv::eigen2cv(P_initial, t);    // eigen 转自cv格式
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  // 相机内参
    /**
     * pts3D： 世界坐标系下的控制点的坐标
     * pts2D： 在图像坐标系下对应的控制点的坐标
     * K 相机内参矩阵
     * D 畸变矩阵
     * rvec 输出的旋转向量。使坐标点从世界坐标系旋转到相机坐标系
     * t 输出的平移向量。使坐标点从世界坐标系平移到相机坐标系
     * 默认使用CV_ITERATIV迭代法
    */
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);    // cv 格式转至 eigen格式, R 矩阵
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);    // cv 格式转至 eigen格式, T 矩阵

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

// 三角化三维坐标，计算feature的depth
void FeatureManager::triangulate(const int frameCnt, const Vector3d Ps[], const Matrix3d Rs[], const Vector3d tic[], const Matrix3d ric[])
{
    for (auto &it_per_id : feature)  // 对于每个id的特征点
    {
        // TODO: 不一致
//        it_per_id.used_num = (int)it_per_id.feature_per_frame.size();
//
//        // used number can be:[1], 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
//        // start frame can be:0, 1, 2, 3, 4, 5, 6, 7, [8, 9, 10]
//        // 此特征点要至少被2帧图像观测到 && 被第一次观测到的帧不能是倒数2帧以后的帧
//        if (!it_per_id.isPriorFeature())
//        {
//            continue;
//        }
        
        // 该id的特征点深度值大于0说明该点被三角化过，在初始化时为-1
        if (it_per_id.estimated_depth > 0)
            continue;

        if (STEREO && it_per_id.feature_per_frame[0].is_stereo)
        {
            /**
             * 双目，利用左右相机计算depth
             */
            int imu_i = it_per_id.start_frame;                    // 坐标定义在imu上 通过imu转换位姿
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];  // 利用imu的位姿计算左相机位姿 T
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];              // 利用imu的位姿计算左相机位姿 R
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);  // svd方法三角化
            Eigen::Vector3d localPoint;     // 左相机坐标系下的坐标
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();  // 深度信息
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if (it_per_id.feature_per_frame.size() > 1)  // Mono
        {
            /**
             * 方法（A）单目，利用第一二帧计算depth
             */
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        
        // TODO: 利用方法（B）计算depth，是不是会更精确？？？
        /**
         * 方法（B）单目，利用观测到词特征点的所有帧计算depth
         */
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        // 首次观测到该特征点的图像在滑动窗口中的帧号和最后一次观测到的图像在滑动窗口中的帧号
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;  // 似乎是[R | T]的形式，是一个位姿
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();  // 单位旋转矩阵
        P0.rightCols<1>() = Eigen::Vector3d::Zero();     // 0平移向量
        
        // 对于观测到该id特征点的每一图像帧
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;   // 观测到该特征点的最后一帧图像在滑动窗口中的帧号

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;

            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;

            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            // 在第一次进入for循环的时候，这个条件成立，这时候循环体都执行完了，
            // continue发挥不了什么作用啊？？？
            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        
        double svd_method = svd_V[2] / svd_V[3];
        it_per_id.estimated_depth = svd_method;  // 似乎是得到了该特征点的深度
        
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if (itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

/**
 * 将特征点进行处理，将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
 */
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}


/**
 * 对于给定id的特征点
 * 计算第2最新帧和第3最新帧之间该特征点的视差（当前帧frame_count是第1最新帧）
 * （需要使用IMU数据补偿因旋转造成的视差）
 */
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax between second last frame and third last frame
    // 第3最新帧
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    // 第2最新帧
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    Vector3d p_i = frame_i.point;
    Vector3d p_j = frame_j.point;
    
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;  // 像素平面坐标
    double v_i = p_i(1) / dep_i;  // 像素平面坐标
    
    double dep_j = p_j(2);
    double u_j = p_j(0) / dep_j;  // 像素平面坐标
    double v_j = p_j(1) / dep_j;  // 像素平面坐标
    
    double du = u_i - u_j;
    double dv = v_i - v_j;
    
    double ans = sqrt(du * du + dv * dv);  // 视差补偿值
    return ans;
}


