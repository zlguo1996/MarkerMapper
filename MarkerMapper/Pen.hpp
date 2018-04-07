//
//  Pen.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef Pen_hpp
#define Pen_hpp

#include "stdafx.h"

using namespace aruco;

struct FramePenInfo{
    cv::Mat rt_mat;
    int id;

    FramePenInfo(Mat mat, int idx){
        rt_mat = mat;
        id = idx;
    }
};

typedef std::map<uint32_t, FramePenInfo> FramePenSet;

class Pen{
    FramePenSet frame_pen_set;
    
public:
    MarkerMap marker_map;   // 存储标定的正十二面体位置
    
    Pen(){
        
    }
    
    Pen(const string& marker_map_path){
        setMarkerMap(marker_map_path);
    }
    
    void setMarkerMap(const string& marker_map_path){
        marker_map.readFromFile(marker_map_path);
    }
    
    void addFrame(FramePenInfo frame_info, int frame_index){
        frame_pen_set.insert(FramePenSet::value_type(frame_index, frame_info));
    }
    
    cv::Mat getFrame(int index){
        cv::Mat mat;
        frame_pen_set[index].rt_mat.copyTo(mat);
        return mat;
    }
    
    bool isValid(){
        return marker_map.mInfoType!=-1;
    }
};

#endif /* Pen_hpp */
