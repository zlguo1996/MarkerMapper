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

namespace DodecaTrackerPlugin {
    DodecaTracker* dodeca_tracker;
    
    extern "C" void EXPORT_API _DodecaTrackerPlugin();
    extern "C" bool EXPORT_API _initCamera(char* camera_parameter_path, int deviceNum);
    extern "C" bool EXPORT_API _initCameraRvecTvec(char* camera_parameter_path, int deviceNum, float* rvec, float* tvec);
    extern "C" bool EXPORT_API _initPen(char* marker_map_path);
    extern "C" bool EXPORT_API _initPenDetector();
    extern "C" bool EXPORT_API _grab();
    extern "C" bool EXPORT_API _detect();
    extern "C" bool EXPORT_API _getPose(float* rvec, float* tvec);
    extern "C" bool EXPORT_API _isValid();
}

