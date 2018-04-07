//
//  PenDetector.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef PenDetector_hpp
#define PenDetector_hpp

#include "stdafx.h"

#include "Camera.hpp"
#include "Pen.hpp"

using namespace std;

// 使用指定的相机对指定的笔进行检测和跟踪
// 存储笔和相机的信息
class PenDetector{
    
public:
    vector<Camera> camera;
    vector<Pen> pen;
    
    PenDetector(){
        
    }
    
    bool addCamera(Camera& c){
        camera.push_back(c);
        return true;
    }
    
    bool addPen(Pen& p){
        pen.push_back(p);
        return true;
    }
    
    bool isValid(){
        // 临时设定只有一个相机和一支笔
        return camera.size()==1&&pen.size()==1;
    }
    
    bool detectOneFrame(){
        if(camera[0].grab()){
            int frame_index = camera[0].retrieve();
            if(camera[0].detectMarkers()<2) return false;
            
            Mat rt_mat;
            if(!camera[0].mmPoseEstimation(rt_mat, pen[0].marker_map)) return false;
            
            rt_mat = camera[0].camera_pose*rt_mat*rt_mat.inv();
            FramePenInfo frame_pen_info(rt_mat, frame_index);
            pen[0].addFrame(frame_pen_info, frame_index);
            
            return true;
        }
        return false;
    }
    
    cv::Mat getLastFramePose(int pen_index){
        return pen[pen_index].getFrame(camera[0].next_frame_index-1);
    }
};

#endif /* PenDetector_hpp */
