//
//  Camera.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "Camera.hpp"

int Camera::max_marker_set_size=65; // 允许的最大map大小
int Camera::max_erase_num=60;          // 缩减到的map大小

// ---------- tools --------
void findBound(const aruco_mm::arucoMarkerSet& marker_set, cv::Point2f& minB, cv::Point2f& maxB){
    minB = Point2f(FLT_MAX, FLT_MAX);
    maxB = Point2f(0, 0);
    
    for (int i=0; i<marker_set.size(); i++) {
        for (int j=0; j<4; j++) {
            const Point2f& point = marker_set[i][j];
            if (point.x<minB.x) minB.x = point.x;
            if (point.x>maxB.x) maxB.x = point.x;
            
            if (point.y<minB.y) minB.y = point.y;
            if (point.y>maxB.y) maxB.y = point.y;
        }
    }
}

// 返回找到的对应的marker的个数
int contoursVelMean(aruco_mm::arucoMarkerSet& marker_set1, aruco_mm::arucoMarkerSet& marker_set2, Point2f mean_vel){
    int count = 0;

    vector<cv::Point2f> ift_contours_vel;
    for(auto marker2:marker_set2){
        if(marker_set1.is(marker2.id)){
            aruco::Marker marker1 = marker_set1.get(marker2.id);
            for(int i=0; i<4; i++) ift_contours_vel.push_back(marker2[i]-marker1[i]);
            
            count++;
        }
    }
    
    Scalar mean_vel_s = cv::mean(ift_contours_vel);
    mean_vel = Point2f(mean_vel_s[0], mean_vel_s[1]);
    return count;
}

// --------- 类方法 -----------

// 检测marker，返回检测到的marker数量
int Camera::detectMarkers(bool is_ift, bool constrain_area){
    auto start = std::chrono::high_resolution_clock::now();
    
    Mat c_current_frame;
    current_frame.copyTo(c_current_frame);
    if(constrain_area) constrainDetectArea(c_current_frame);
    aruco_mm::arucoMarkerSet markers = marker_detector.detect(c_current_frame);
    
    auto finish1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed1 = finish1 - start;
    std::cout << "  -dm_d " << elapsed1.count() << " s\n";
    
    if(is_ift && markers.size()<2 && marker_set.count(next_frame_index-2)){
        aruco_mm::arucoMarkerSet ift_markers = detectInterframeMarkers(markers);
        markers.insert(markers.end(), ift_markers.begin(), ift_markers.end());
    }
    marker_set.insert(pair<uint32_t, aruco_mm::arucoMarkerSet>(next_frame_index-1, markers));
    auto finish2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = finish2 - finish1;
    std::cout << "  -dm_ift " << elapsed2.count() << " s\n";
    
    // 限制marker_set大小
    if(marker_set.size()>max_marker_set_size){
        while(marker_set.size()>max_erase_num){
            marker_set.erase(marker_set.begin());
        }
    }
    
    return marker_set[next_frame_index-1].size();
}

// 限定marker的检测范围
void Camera::constrainDetectArea(Mat& frame){
    if (!(marker_set.count(next_frame_index-2)&&marker_set.count(next_frame_index-3))) return;
    if (marker_set[next_frame_index-2].empty() || marker_set[next_frame_index-3].empty()) return;
    
    // 估计本帧的位置
    Point2f minBound, maxBound;
    findBound(marker_set[next_frame_index-2], minBound, maxBound);
    // 加上速度
    Point2f vel2;
    contoursVelMean(marker_set[next_frame_index-3], marker_set[next_frame_index-2], vel2);
    minBound += vel2;
    maxBound += vel2;
    // 加上加速度
    if (marker_set.count(next_frame_index-4) && !marker_set[next_frame_index-4].empty()) {
        Point2f vel1;
        contoursVelMean(marker_set[next_frame_index-4], marker_set[next_frame_index-3], vel1);
        minBound += 0.5*(vel2-vel1);
        maxBound += 0.5*(vel2-vel1);
    }
    
    // 获得区域
    // 边长三倍于估计位置（论文为四倍区域？）
    Point2f diff = maxBound- minBound;
    Point2f luBound = minBound-1.5f*diff, rbBound = maxBound+1.5f*diff;
    // 区域交
    Rect2i c_area((Point2i)luBound, (Point2i)rbBound);
    Rect2i t_area(Point2i(0, 0), frame.size());
    Rect2i final_area = c_area & t_area;
    if (final_area.width==0 || final_area.height==0) return;
    
    // 获得图片
    Mat new_frame = Mat(frame.rows, frame.cols, CV_8UC3, Scalar(0, 0, 0));
    frame(final_area).copyTo(new_frame(final_area));
    new_frame.copyTo(frame);
}

// 检测帧间marker
aruco_mm::arucoMarkerSet Camera::detectInterframeMarkers(aruco_mm::arucoMarkerSet& detected_markers){
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
            if(ift_erase_idx.empty()||ift_erase_idx.back()!=i/4) ift_erase_idx.push_back(i/4);
        }
    }
    for(int i=ift_erase_idx.size()-1; i>=0; i--) ift_markers.erase(ift_markers.begin()+ift_erase_idx[i]);
    
    return ift_markers;
}
