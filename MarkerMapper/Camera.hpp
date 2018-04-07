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
    aruco::CameraParameters camera_parameters;  // 相机参数
    cv::VideoCapture video_capture;   // 现实中的相机
    aruco::MarkerDetector marker_detector;   // marker检测器
    aruco::MarkerMapPoseTracker mm_pose_tracker;    // marker位置追踪
    
    std::map<uint32_t, aruco_mm::arucoMarkerSet> marker_set;   // 存储每帧检测到的marker
    
public:
    cv::Mat current_frame;
    cv::Mat last_frame;
    int next_frame_index;
    
    cv::Mat camera_pose; // 相机在世界坐标系中的pose（相机坐标系转化为世界坐标系）
    
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
    void setCameraPose(const Mat& rt){
        camera_pose = rt;
    }
    void setMarkerDetector(const aruco::MarkerDetector& md){
        marker_detector.setParameters(md.getParameters());
    }
    
    bool isValid(){
        return video_capture.isOpened()&&camera_parameters.isValid()&&!camera_pose.empty();
    }
    
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
    int detectMarkers(){
        aruco_mm::arucoMarkerSet markers = marker_detector.detect(current_frame);
        aruco_mm::arucoMarkerSet ift_markers = detectInterframeMarkers(markers);
        markers.insert(markers.end(), ift_markers.begin(), ift_markers.end());
        marker_set.insert(pair<uint32_t, aruco_mm::arucoMarkerSet>(next_frame_index-1, markers));
        return marker_set[next_frame_index-1].size();
    }
    
    // 追踪markermap相对于相机的位置，返回是否检测成功
    bool mmPoseEstimation(Mat& rt_mat, const aruco::MarkerMap& mm){
        mm_pose_tracker.setParams(camera_parameters, mm);
        if(!mm_pose_tracker.estimatePose(marker_set[next_frame_index-1])) return false;
        rt_mat = mm_pose_tracker.getRTMatrix();
        return true;
    }
    
    // =============== 以下为一些工具 ================
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
    // 检测帧间marker
    aruco_mm::arucoMarkerSet detectInterframeMarkers(aruco_mm::arucoMarkerSet& detected_markers){
        if(next_frame_index<2) return aruco_mm::arucoMarkerSet();
        
        aruco_mm::arucoMarkerSet ift_markers;
        vector<cv::Point2f> ift_contours_vel;
        for(auto marker:marker_set[next_frame_index-2]){
            if(!detected_markers.is(marker.id)){
                aruco::Marker dm;
                vector<uchar> status;
                vector<float> error;
                calcOpticalFlowPyrLK(last_frame, current_frame, marker, dm, status, error);
                dm.id = marker.id;
                dm.ssize = marker.ssize;
                dm.dict_info = marker.dict_info;
                
                // 除去光流法失败的marker
                if(find(status.begin(), status.end(), 0)!=status.end()) continue;
                
                for(int i=0; i<4; i++) ift_contours_vel.push_back(dm[i]-marker[i]);
                
                ift_markers.push_back(dm);
            }
        }
        
        // 异常值移除(除去大于均值两个标准差的marker)
        Scalar mean, stddev;
        cv::meanStdDev(ift_contours_vel, mean, stddev);
        cv::Point2f mean_p(mean[0], mean[1]), stddev_p(stddev[0], stddev[1]);
        vector<int> ift_erase_idx;
        for(int i=0; i<ift_contours_vel.size(); i++){
            Point2f p = ift_contours_vel[i];
            float dist2 = (p-mean_p).dot(p-mean_p);
            if(dist2>4.0*stddev_p.dot(stddev_p)){
                if(ift_erase_idx.empty()||ift_erase_idx.back()!=i) ift_erase_idx.push_back(i/4);
            }
        }
        for(int i=ift_erase_idx.size()-1; i>=0; i--) ift_markers.erase(ift_markers.begin()+ift_erase_idx[i]);
        
        return ift_markers;
    }
};

#endif /* Camera_hpp */
