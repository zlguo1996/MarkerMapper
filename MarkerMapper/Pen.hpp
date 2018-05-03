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

typedef std::map<uint32_t, cv::Mat> FramePenSet;

class Pen{
    FramePenSet frame_pen_set;
    static int max_frame_pen_set_size; // 允许的最大map大小
    static int max_erase_num;          // 缩减到的map大小
    
public:
    MarkerMap marker_map;   // 存储标定的正十二面体位置
    Mat pentip_pose;    // 存储笔尖的位置(4*4 float）
    Mat dodeca_center_pose; // 存储正十二面体中心的位置(4*4 float）
    
    Pen(){
    }
    
    Pen(const string& marker_map_path){
        setMarkerMap(marker_map_path);
    }
    
    void setMarkerMap(const string& marker_map_path){
        marker_map.readFromFile(marker_map_path);
    }
    
    void setPenTip(const string& pentip_position_path){
        FileStorage fs(pentip_position_path, FileStorage::READ);
        fs["pentip_position"] >> pentip_pose;
        fs.release();
        assert(!pentip_pose.empty());
    }
    
    void setDodecaCenter(const string& dodeca_center_path){
        FileStorage fs(dodeca_center_path, FileStorage::READ);
        fs["pentip_position"] >> dodeca_center_pose;
        fs.release();
        assert(!dodeca_center_pose.empty());
    }
    
    void addFrame(cv::Mat rt_mat, int frame_index){
        frame_pen_set.insert(FramePenSet::value_type(frame_index, rt_mat));
        
        if(frame_pen_set.size()>max_frame_pen_set_size) {
            while (frame_pen_set.size()>max_erase_num) {
                frame_pen_set.erase(frame_pen_set.begin());
            }
        }
    }
    
    cv::Mat getFrame(int index){
        cv::Mat mat;
        frame_pen_set[index].copyTo(mat);
        return mat;
    }
    
    bool isValid(){
        return marker_map.mInfoType!=-1;
    }
};

#endif /* Pen_hpp */
