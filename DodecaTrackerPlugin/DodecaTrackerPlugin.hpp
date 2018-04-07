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
    extern DodecaTracker dodeca_tracker;
    extern "C" bool EXPORT_API initCamera(char* camera_parameter_path, int deviceNum=0){
        string camera_parameter_path_s = camera_parameter_path;
        dodeca_tracker.initCamera(camera_parameter_path_s, deviceNum);
    }
    extern "C" bool EXPORT_API initCameraRvecTvec(char* camera_parameter_path, float* rvec, float* tvec, int deviceNum=0){
        string camera_parameter_path_s = camera_parameter_path;
        Mat cam_mat;
        
        
    }
}


