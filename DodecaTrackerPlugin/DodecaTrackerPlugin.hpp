//
//  DodecaTrackerPlugin.hpp
//  DodecaTrackerPlugin
//
//  Created by 郭子乐 on 2018/3/31.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#pragma once

#if UNITY_METRO
#define EXPORT_API __declspec(dllexport) __stdcall
#elif UNITY_WIN
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

#include "../MarkerMapper/DodecaTracker.hpp"
#include "../MarkerMapper/Libraries/my_lib/Tools.hpp"

struct Color32
{
    uchar r;
    uchar g;
    uchar b;
    uchar a;
};

namespace DodecaTrackerPlugin {
    DodecaTracker* dodeca_tracker;
    
    cv::Mat CVToU3d;
    
    // ------- 基本函数 ------
    extern "C" void EXPORT_API _DodecaTrackerPlugin();
    extern "C" bool EXPORT_API _reset();
    extern "C" bool EXPORT_API _isValid();
    //初始化
    extern "C" bool EXPORT_API _initCamera(char* camera_parameter_path, int deviceNum);
    extern "C" bool EXPORT_API _initCameraRvecTvec(char* camera_parameter_path, int deviceNum, float* rvec, float* tvec);
    extern "C" bool EXPORT_API _initPen(char* marker_map_path);
    extern "C" bool EXPORT_API _initPenDetector();
    //检测
    extern "C" bool EXPORT_API _grab();
    extern "C" bool EXPORT_API _detect();
    extern "C" bool EXPORT_API _getPose(float* rvec, float* tvec);          // deprecated
    extern "C" bool EXPORT_API _getPoseM(float* rt_mat);                    // 4*4 行主序
    
    // ------- 相机标定 -------
    extern "C" int  EXPORT_API _detectMarkers();
    extern "C" bool EXPORT_API _calibrateCameraPose(char* calib_marker_map_path);
    extern "C" bool EXPORT_API _getCameraPose(float* rvec, float* tvec);    // deprecated
    extern "C" bool EXPORT_API _getCameraPoseM(float* rt_mat);              // 4*4 行主序
    
    // ------- 笔尖和正十二面体中心 ------
    extern "C" bool EXPORT_API _setPenTip(char* file_path);
    extern "C" bool EXPORT_API _setPenDodecaCenter(char* file_path);
    extern "C" bool EXPORT_API _getPenTipPosition(float* tvec);             // deprecated
    extern "C" bool EXPORT_API _getPenDodecaCenterPosition(float* tvec);    // deprecated
    extern "C" bool EXPORT_API _setPenTipM(float* rt_mat);                  // 4*4 行主序
    extern "C" bool EXPORT_API _setPenDodecaCenterM(float* rt_mat);         // 4*4 行主序
    extern "C" bool EXPORT_API _savePenTipPose(float* rt_mat, char* file_path);
    extern "C" bool EXPORT_API _getPenTipPose(float* rt_mat);               // 4*4 行主序
    extern "C" bool EXPORT_API _getPenDodecaCenterPose(float* rt_mat);      // 4*4 行主序
    
    // -------- 新函数测试 ---------
    extern "C" void EXPORT_API processImage(Color32* raw, int width, int height, int id);
}

