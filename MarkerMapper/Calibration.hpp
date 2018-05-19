//
//  Calibration.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef Calibration_hpp
#define Calibration_hpp

#include "stdafx.h"
#include <unistd.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "DodecaTracker.hpp"

// ----------- 打印 markers -------
bool printMarkers(const string& file_path, float side_length_mm, float marker_length_mm);

// ----------- tools -------------
// 读取文件夹中以自然数命名的图片
bool readImages(const string& photo_path, vector<Mat>& images);

// ------------ 相机标定 ------------
void calibrateCameraWithImages(const string& file_path, const string& calibration_photo_path, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map="Calibration/input/aruco_calibration_grid_board_a4.yml");
void calibrateCameraWithVideo(const string& file_path, const string& calibration_video_path, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map="Calibration/input/aruco_calibration_grid_board_a4.yml");

// ---------- 正十二面体标定 ------------
void calibrateDodecaWithImages(const string& file_path_base_name, const string& calibration_photo_path, float calibration_marker_size, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary);

// ------------ 笔尖标定 -------------
// 定义投影误差
// 用于标定笔尖
typedef double DOF_6[6];    // r1, r2, r3, t1, t2, t3
struct ReprojectionErrorPtClb{
    ReprojectionErrorPtClb(const DOF_6 const my_pose1, const DOF_6 const my_pose2){
        memcpy(pose1, my_pose1, 6*sizeof(double));
        memcpy(pose2, my_pose2, 6*sizeof(double));
    }
    
    template <typename T>
    bool operator()(const T* const ptPst, T* residuals) const {
        T pose1T[6] = {
            T(pose1[0]),T(pose1[1]),T(pose1[2]),T(pose1[3]),T(pose1[4]),T(pose1[5])
        };
        T pose2T[6] = {
            T(pose2[0]),T(pose2[1]),T(pose2[2]),T(pose2[3]),T(pose2[4]),T(pose2[5])
        };
        
        // apply rotation
        T p1[3], p2[3];
        ceres::AngleAxisRotatePoint(pose1T, ptPst, p1);
        ceres::AngleAxisRotatePoint(pose2T, ptPst, p2);
        
        // apply translation
        for (int i=0; i<3; i++){
            p1[i] += pose1T[3+i];
            p2[i] += pose2T[3+i];
        }
        
        // error
        residuals[0] = p1[0]-p2[0];
        residuals[1] = p1[1]-p2[1];
        residuals[2] = p1[2]-p2[2];
        
        return true;
    }
    
    static ceres::CostFunction* Create(const DOF_6 const my_pose1, const DOF_6 const my_pose2){
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorPtClb, 3, 3>(new ReprojectionErrorPtClb(my_pose1, my_pose2)));
    }
    
    DOF_6 pose1, pose2;
};
void calibratePentip(const string& file_path, const string& calibration_photo_path, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary, const aruco::MarkerMap& marker_map);
void visualizePoints(const string& file_path, const double* points, int point_num);
#endif /* Calibration_hpp */
