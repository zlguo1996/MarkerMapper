//
//  Camera.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include "stdafx.h"

#include <chrono>

//struct MarkerInfo: public vector<aruco::Marker>{
//    MarkerInfo() {}
//    MarkerInfo(const vector<aruco::Marker>& markers) : vector<aruco::Marker>(markers){}
//
//    aruco::Marker findMarkerId(const aruco::Marker& marker) const {
//        for(auto m:*this){
//            if(m.id==marker.id) return m;
//        }
//        return aruco::Marker();
//    }
//};

class Camera{
    cv::VideoCapture video_capture;   // 现实中的相机
    aruco::MarkerDetector marker_detector;   // marker检测器
    aruco::MarkerMapPoseTracker mm_pose_tracker;    // marker位置追踪
    
    std::map<uint32_t, aruco_mm::arucoMarkerSet> marker_set;   // 存储每帧检测到的marker
    static int max_marker_set_size; // 允许的最大map大小
    static int max_erase_num;          // 缩减到的map大小
    
public:
    aruco::CameraParameters camera_parameters;  // 相机参数
    cv::Mat current_frame;
    cv::Mat last_frame;
    int next_frame_index;
    
    cv::Mat camera_pose; // 相机在世界坐标系中的pose（相机坐标系转化为世界坐标系）
    
    // ------------ 初始化 ----------------
    Camera(){
        
    }
    
    Camera(const string& camera_parameter_path, const VideoCapture& vc, const Mat& rt, const aruco::MarkerDetector& md){
        setCameraParameter(camera_parameter_path);
        setVideoCapture(vc);
        setCameraPose(rt);
        setMarkerDetector(md);
        
        next_frame_index = 0;
    }
    
    void setCameraParameter(const string& camera_parameter_path){
        camera_parameters.readFromXMLFile(camera_parameter_path);
    }
    void setVideoCapture(const VideoCapture& vc){
        video_capture = vc;
    }
    void setVideoCaptureParameters1080p(){
        video_capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
        video_capture.set(CAP_PROP_FRAME_WIDTH, 1920.0);　　//设置摄像头采集图像分辨率
        video_capture.set(CAP_PROP_FRAME_HEIGHT, 1080.0);
    }
    void setVideoCaptureParameters720p(){
        video_capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
        video_capture.set(CAP_PROP_FRAME_WIDTH, 1280.0);　　//设置摄像头采集图像分辨率
        video_capture.set(CAP_PROP_FRAME_HEIGHT, 720.0);
    }
    void setVideoCapture(const int& vc){
        video_capture = cv::VideoCapture(vc);
    }
    void setVideoCapture(const string& vc){
        video_capture = cv::VideoCapture(vc);
    }
    void setCameraPose(const Mat& rt){
        camera_pose = rt;
    }
    void setMarkerDetector(const aruco::MarkerDetector& md){
        marker_detector.setParameters(md.getParameters());
        //marker_detector.getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    }
    
    // ------------- 基本API ---------------
    bool isValid(){
        return video_capture.isOpened()&&camera_parameters.isValid()&&!camera_pose.empty();
    }
    
    bool calibratePose(string axis_marker_map_file_path){
        aruco::MarkerMapPoseTracker mmpt;
        aruco::MarkerMap mm;
        mm.readFromFile(axis_marker_map_file_path);
        mmpt.setParams(camera_parameters, mm);
        if(!mmpt.estimatePose(marker_set[next_frame_index-1])) return false;;
        camera_pose = mmpt.getRTMatrix().inv();
        return true;
    }
    
    bool reset(){
        video_capture.release();
        return true;
    }
    
    // --------------- 追踪 -------------------
    bool grab(){
        return video_capture.grab();
    }
    
    // 获得下一帧，返回这一帧id
    int retrieve(){
        if(!current_frame.empty()) current_frame.copyTo(last_frame);
        video_capture.retrieve(current_frame);
        return next_frame_index++;
    }
    
    // 检测marker，返回检测到的marker数量
    int detectMarkers(bool is_ift=true);
    
    // 追踪markermap相对于相机的位置，返回是否检测成功
    bool mmPoseEstimation(Mat& rt_mat, const aruco::MarkerMap& mm){
        mm_pose_tracker.reset();
        mm_pose_tracker.setParams(camera_parameters, mm);
        assert(mm_pose_tracker.isValid());
        if(!mm_pose_tracker.estimatePose(marker_set[next_frame_index-1])) return false;
        rt_mat = mm_pose_tracker.getRTMatrix();
        
        return true;
    }
    
    // --------------- 可视化工具 --------------
    // 绘制当前帧检测到的marker
    void drawDetectedMarkers(Mat& img){
        aruco_mm::arucoMarkerSet markers = marker_set[next_frame_index-1];
        for(auto marker:markers) marker.draw(img);
    }
    
    // 通过恢复的位置，绘制3d坐标
    void draw3DAxis(Mat& img, float axisLength){
        aruco::CvDrawingUtils::draw3dAxis(img, camera_parameters, mm_pose_tracker.getRvec(), mm_pose_tracker.getTvec(), axisLength);
    }
    
private:
    // --------------- 私有函数 ----------------
    // 限定marker的检测范围
    void constrainDetectArea(Mat& frame);
    
    // 检测帧间marker
    aruco_mm::arucoMarkerSet detectInterframeMarkers(aruco_mm::arucoMarkerSet& detected_markers);
};

#endif /* Camera_hpp */
