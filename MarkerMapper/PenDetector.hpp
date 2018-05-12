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
    bool detectOneFrame();
};

#endif /* PenDetector_hpp */
