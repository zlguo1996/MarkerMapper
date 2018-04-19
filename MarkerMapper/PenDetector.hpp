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

// 使用相机对笔进行检测和跟踪
class PenDetector{
    
public:
    Camera* camera;
    Pen* pen;
    
    PenDetector(){
        camera = NULL;
        pen = NULL;
    }
    
    PenDetector(Camera* c, Pen* p){
        camera = c;
        pen = p;
    }
    
    bool isValid(){
        // 临时设定只有一个相机和一支笔
        return camera->isValid() && pen->isValid();
    }
    
    bool grabOneFrame(){
        return camera->grab();
    }
    bool detectOneFrame(){
        int frame_index = camera->retrieve();
        if(camera->detectMarkers()<2) return false;
        
        Mat rt_mat;
        if(!camera->mmPoseEstimation(rt_mat, pen->marker_map)) return false;
        
        rt_mat = camera->camera_pose*rt_mat;
        pen->addFrame(rt_mat, frame_index);
        
        return true;
    }
};

#endif /* PenDetector_hpp */
