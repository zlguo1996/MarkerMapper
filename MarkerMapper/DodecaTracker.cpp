//
//  DodecaTracker.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/4/5.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "DodecaTracker.hpp"

//-------------- 参数 -------------------------
// 相机标定变量
float DodecaTracker::calibration_marker_size = 0.018;    //标定用marker的边长
// 正十二面体变量
aruco::Dictionary::DICT_TYPES DodecaTracker::dictionary = aruco::Dictionary::ARUCO_MIP_36h12;  //正十二面体表面marker所属的字典
float DodecaTracker::dodeca_marker_size = 0.014;       //正十二面体表面marker边长
