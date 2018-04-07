//
//  DodecaTracker.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/4/5.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef DodecaTracker_hpp
#define DodecaTracker_hpp

#include "stdafx.h"

#include "Pen.hpp"
#include "Camera.hpp"
#include "PenDetector.hpp"

using namespace std;

// 管理相机和笔和追踪
class DodecaTracker{
private:
    // 相机参数
    string camera_parameter_path;
    int video_capture_index;
    cv::Mat camera_pose;
    // 笔参数
    string marker_map_path;
    
    bool haveInitCamera;
    bool haveInitPen;

    PenDetector pen_detector;
    
    bool getCamera(Camera& camera){
        assert(haveInitCamera);
        
        aruco::MarkerDetector md;
        md.setDictionary(dictionary);
        md.getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
        
        camera.setCameraParameter(camera_parameter_path);
        camera.setVideoCapture(video_capture_index);
        camera.setCameraPose(camera_pose);
        camera.setMarkerDetector(md);
        
        return true;
    }
    bool getPen(Pen& pen){
        assert(haveInitPen);
        
        pen.setMarkerMap(marker_map_path);
        
        return true;
    }
    
public:
    //---------------- 参数 -------------------------
    // marker 参数
    // 相机标定变量
    static float calibration_marker_size;    //标定用marker的边长
    // 正十二面体变量
    static aruco::Dictionary::DICT_TYPES dictionary;  //正十二面体表面marker所属的字典
    static float dodeca_marker_size;       //正十二面体表面marker边长
    
    DodecaTracker(){
        haveInitCamera = false;
        haveInitPen = false;
    }
    
    bool initCamera(string camera_parameter_path, int video_capture_index=0, Mat camera_pose = cv::Mat::eye(4, 4, CV_32F)){
        fstream file;
        file.open(camera_parameter_path);
        if(!file.is_open()) return false;
        
        this->camera_parameter_path = camera_parameter_path;
        this->video_capture_index = video_capture_index;
        this->camera_pose = camera_pose;
        
        haveInitCamera = true;
        return true;
    }
    bool initPen(string marker_map_path){
        fstream file;
        file.open(marker_map_path);
        if(!file.is_open()) return false;
        
        this->marker_map_path = marker_map_path;
        
        haveInitPen = true;
        return true;
    }
    bool initPenDetector(){
        assert(haveInitPen&&haveInitCamera);
        
        Camera camera;
        getCamera(camera);
        Pen pen;
        getPen(pen);
        
        if(!(camera.isValid()&&pen.isValid())) return false;
        
        pen_detector.addCamera(camera);
        pen_detector.addPen(pen);
        
        return pen_detector.isValid();
    }
    
    // 检测当前帧
    bool detect(){
        return pen_detector.detectOneFrame();
    }
    
    // 获得当前帧指定笔的世界坐标的pose
    cv::Mat getPose(int pen_index=0){
        return pen_detector.getLastFramePose(pen_index);
    }
    
    bool isValid(){
        return pen_detector.isValid();
    }
};

#endif /* DodecaTracker_hpp */
