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

#include <chrono>

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
        cout << "--------- frame id : " << camera->next_frame_index << "----------" << endl;
        auto start = std::chrono::high_resolution_clock::now();
        
        int frame_index = camera->retrieve();
        auto finish1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed1 = finish1 - start;
        std::cout << " - " << elapsed1.count() << " s\n";
        
        if(camera->detectMarkers(true, true)<2) return false;
        auto finish2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed2 = finish2 - finish1;
        std::cout << " - " << elapsed2.count() << " s\n";
        
        Mat rt_mat;
        if(!camera->mmPoseEstimation(rt_mat, pen->marker_map)) return false;
        auto finish3 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed3 = finish3 - finish2;
        std::cout << " - " << elapsed3.count() << " s\n";
        
        rt_mat = camera->camera_pose*rt_mat;
        pen->addFrame(rt_mat, frame_index);
        
        auto finish4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed4 = finish4 - start;
        std::cout << "+ " << elapsed4.count() << " s\n";
        cout << "-------------------" << endl;
        cout << endl;
        
        return true;
    }
};

#endif /* PenDetector_hpp */
