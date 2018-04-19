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
    
    extern "C" void EXPORT_API _DodecaTrackerPlugin();
    extern "C" bool EXPORT_API _reset();
    extern "C" bool EXPORT_API _initCamera(char* camera_parameter_path, int deviceNum);
    extern "C" bool EXPORT_API _initCameraRvecTvec(char* camera_parameter_path, int deviceNum, float* rvec, float* tvec);
    extern "C" bool EXPORT_API _initPen(char* marker_map_path);
    extern "C" bool EXPORT_API _initPenDetector();
    extern "C" bool EXPORT_API _grab();
    extern "C" bool EXPORT_API _detect();
    extern "C" int  EXPORT_API _detectMarkers();
    extern "C" bool EXPORT_API _calibrateCameraPose(char* calib_marker_map_path);
    extern "C" bool EXPORT_API _getCameraPose(float* rvec, float* tvec);
    extern "C" bool EXPORT_API _getPose(float* rvec, float* tvec);
    extern "C" bool EXPORT_API _isValid();
    
    extern "C" void EXPORT_API processImage(Color32* raw, int width, int height, int id);
}

