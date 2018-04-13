//
//  DodecaTrackerPlugin.cpp
//  DodecaTrackerPlugin
//
//  Created by 郭子乐 on 2018/3/31.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "DodecaTrackerPlugin.hpp"

//DodecaTracker DodecaTrackerPlugin::dodeca_tracker = DodecaTracker();

using namespace DodecaTrackerPlugin;

void EXPORT_API DodecaTrackerPlugin::_DodecaTrackerPlugin(){
    dodeca_tracker = new DodecaTracker();
}

bool EXPORT_API DodecaTrackerPlugin::_initCamera(char* camera_parameter_path, int deviceNum){
    string camera_parameter_path_s = camera_parameter_path;
    return dodeca_tracker->initCamera(camera_parameter_path_s, deviceNum);
}

bool EXPORT_API DodecaTrackerPlugin::_initCameraRvecTvec(char* camera_parameter_path, int deviceNum, float* rvec, float* tvec){
    string camera_parameter_path_s = camera_parameter_path;
    Mat cam_mat;
    getViewMatrixFromRvecTvec(rvec, tvec, cam_mat);
    return dodeca_tracker->initCamera(camera_parameter_path_s, deviceNum, cam_mat);
}

bool EXPORT_API DodecaTrackerPlugin::_initPen(char* marker_map_path){
    string marker_map_path_s = marker_map_path;
    return dodeca_tracker->initPen(marker_map_path_s);
}

bool EXPORT_API DodecaTrackerPlugin::_initPenDetector(){
    return dodeca_tracker->initPenDetector();
}

bool EXPORT_API DodecaTrackerPlugin::_grab(){
    return dodeca_tracker->grab();
}

bool EXPORT_API DodecaTrackerPlugin::_detect(){
    return dodeca_tracker->detect();
}

bool EXPORT_API DodecaTrackerPlugin::_getPose(float* rvec, float* tvec){
    cv::Mat mat = dodeca_tracker->getPose();
    cv::Mat rv, tv;
    getRvecTvecFromViewMatrix(mat, rv, tv);
    
    rvec[0] = rv.at<float>(0,0);
    rvec[1] = rv.at<float>(1,0);
    rvec[2] = rv.at<float>(2,0);
    
    tvec[0] = tv.at<float>(0,0);
    tvec[1] = tv.at<float>(1,0);
    tvec[2] = tv.at<float>(2,0);
    
    return true;
}

bool EXPORT_API DodecaTrackerPlugin::_isValid(){
    return dodeca_tracker->isValid();
}

