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
    
    CVToU3d = cv::Mat::zeros(4, 4, CV_32F);
    CVToU3d.at<float>(0, 1) = 1.0f;
    CVToU3d.at<float>(1, 2) = 1.0f;
    CVToU3d.at<float>(2, 0) = -1.0f;
    CVToU3d.at<float>(3, 3) = 1.0f;
}

bool EXPORT_API DodecaTrackerPlugin::_reset(){
    return dodeca_tracker->reset();
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

extern "C" bool EXPORT_API _initCameraS(char* camera_parameter_path, char* file_path){
    string camera_parameter_path_s = camera_parameter_path;
    return dodeca_tracker->initCamera(camera_parameter_path_s, file_path);
}

extern "C" bool EXPORT_API _initCameraSRvecTvec(char* camera_parameter_path, char* file_path, float* rvec, float* tvec){
    string camera_parameter_path_s = camera_parameter_path;
    Mat cam_mat;
    getViewMatrixFromRvecTvec(rvec, tvec, cam_mat);
    return dodeca_tracker->initCamera(camera_parameter_path_s, file_path, cam_mat);
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

int  EXPORT_API DodecaTrackerPlugin::_detectMarkers(){
    return dodeca_tracker->detectMarkers();
}

bool EXPORT_API DodecaTrackerPlugin::_calibrateCameraPose(char* calib_marker_map_path){
    string calib_marker_map_path_s = calib_marker_map_path;
    return dodeca_tracker->calibrateCameraPose(calib_marker_map_path_s);
}

bool EXPORT_API DodecaTrackerPlugin::_getCameraPose(float* rvec, float* tvec){
    cv::Mat mat = CVToU3d*dodeca_tracker->getCameraPose();
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
bool EXPORT_API DodecaTrackerPlugin::_getCameraPoseM(float* rt_mat){
    cv::Mat mat = CVToU3d*dodeca_tracker->getCameraPose();
    if (mat.isContinuous()) {
        memcpy(rt_mat, mat.data, 16*sizeof(float));
        return true;
    }else{
        return false;
    }
}

bool EXPORT_API DodecaTrackerPlugin::_getPose(float* rvec, float* tvec){
    cv::Mat mat = CVToU3d*dodeca_tracker->getPose();
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
bool EXPORT_API DodecaTrackerPlugin::_getPoseM(float* rt_mat){
    cv::Mat mat = CVToU3d*dodeca_tracker->getPose();
    if (mat.isContinuous()) {
        memcpy(rt_mat, mat.data, 16*sizeof(float));
        return true;
    }else{
        return false;
    }
}

bool EXPORT_API DodecaTrackerPlugin::_isValid(){
    return dodeca_tracker->isValid();
}

bool EXPORT_API DodecaTrackerPlugin::_setPenTip(char* file_path){
    string pt_file_path = file_path;
    return dodeca_tracker->setPenTip(pt_file_path);
}
bool EXPORT_API DodecaTrackerPlugin::_setPenDodecaCenter(char* file_path){
    string dc_file_path = file_path;
    return dodeca_tracker->setDodecaCenter(dc_file_path);
}
bool EXPORT_API DodecaTrackerPlugin::_getPenTipPosition(float* tvec){
    cv::Mat rt_mat;
    bool success = dodeca_tracker->getPenTipPose(rt_mat);
    if(!success) return false;
    
    tvec[0] = rt_mat.at<float>(0, 3);
    tvec[1] = rt_mat.at<float>(1, 3);
    tvec[2] = rt_mat.at<float>(2, 3);
    return true;
}
bool EXPORT_API DodecaTrackerPlugin::_getPenDodecaCenterPosition(float* tvec){
    cv::Mat rt_mat;
    bool success = dodeca_tracker->getDodecaCenterPose(rt_mat);
    if(!success) return false;
    
    tvec[0] = rt_mat.at<float>(0, 3);
    tvec[1] = rt_mat.at<float>(1, 3);
    tvec[2] = rt_mat.at<float>(2, 3);
    return true;
}
bool EXPORT_API DodecaTrackerPlugin::_setPenTipM(float* rt_mat){
    Mat mat(4, 4, CV_32F);
    memcpy(mat.data, rt_mat, 16*sizeof(float));
    return dodeca_tracker->setPenTip(mat);
}
bool EXPORT_API DodecaTrackerPlugin::_setPenDodecaCenterM(float* rt_mat){
    Mat mat(4, 4, CV_32F);
    memcpy(mat.data, rt_mat, 16*sizeof(float));
    return dodeca_tracker->setDodecaCenter(mat);
}
bool EXPORT_API DodecaTrackerPlugin::_savePenTipPose(float* rt_mat, char* file_path){
    Mat mat(4, 4, CV_32F);
    memcpy(mat.data, rt_mat, 16*sizeof(float));
    string pt_file_path = file_path;
    FileStorage fs(pt_file_path, FileStorage::WRITE);
    fs << "pentip_position" << mat;
    fs.release();
    
    return true;
}

bool EXPORT_API DodecaTrackerPlugin::_getPenTipPose(float* rt_mat){
    cv::Mat mat;
    bool success = dodeca_tracker->getPenTipPose(mat);
    if(!success) return false;
    
    memcpy(rt_mat, mat.data, 16*sizeof(float));
    return true;
}
bool EXPORT_API DodecaTrackerPlugin::_getPenDodecaCenterPose(float* rt_mat){
    cv::Mat mat;
    bool success = dodeca_tracker->getDodecaCenterPose(mat);
    if(!success) return false;
    
    memcpy(rt_mat, mat.data, 16*sizeof(float));
    return true;
}
bool EXPORT_API DodecaTrackerPlugin::_getPoseFromFile(float* rt_mat, char* file_path){
    string file_path_s = file_path;
    FileStorage fs(file_path_s, FileStorage::READ);
    Mat pose;
    fs["pentip_position"] >> pose;
    fs.release();
    
    memcpy(rt_mat, pose.data, 16*sizeof(float));
    return true;
}

void EXPORT_API DodecaTrackerPlugin::processImage(Color32* raw, int width, int height, int id){
    Mat frame(height, width, CV_8UC4, raw);
    string file_path = "/Users/guozile/Desktop/frames";
    file_path = file_path + "/" + to_string(id) + ".jpg";
    Mat frame2;
    cv::cvtColor(frame, frame, COLOR_RGBA2BGRA);
    cv::cvtColor(frame, frame2, COLOR_BGRA2BGR);
    imwrite(file_path, frame2);
}

