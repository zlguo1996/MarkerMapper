//
//  Calibration.hpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#ifndef Calibration_hpp
#define Calibration_hpp

#include "stdafx.h"

// 相机标定
void calibrateCameraWithImages(const string& file_path, const string& calibration_photo_path, int image_num, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map="Calibration/input/aruco_calibration_grid_board_a4.yml");
void calibrateCameraWithVideo(const string& file_path, const string& calibration_video_path, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map="Calibration/input/aruco_calibration_grid_board_a4.yml");

// 正十二面体标定
void calibrateDodecaWithImages(const string& file_path_base_name, const string& calibration_photo_path, int image_num, float calibration_marker_size, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary);

#endif /* Calibration_hpp */
