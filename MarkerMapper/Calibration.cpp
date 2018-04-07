//
//  Calibration.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "Calibration.hpp"

// 相机标定
void calibrateCameraWithImages(const string& file_path, const string& calibration_photo_path, int image_num, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map){
    // 获得图像和marker位置
    aruco::MarkerDetector marker_detector(aruco::Dictionary::ARUCO_MIP_36h12);  //检测marker
    cv::Size img_size(image_width, image_height);
    
    aruco::Calibrator calibrator;               //标定
    aruco::CameraParameters camp;               //相机参数
    calibrator.setParams(img_size, calibration_marker_size, calibration_marker_map);
    
    for (int i=0; i<image_num; i++) {
        string path = calibration_photo_path+"/"+to_string(i)+".jpg";
        cv::Mat img = cv::imread(path);
        assert(!img.empty());
        vector<aruco::Marker> markers =  marker_detector.detect(img);   //检测marker
        calibrator.addView(markers);
    }
    
    cout << "calibrate camera" << endl;
    cout << "    num of views: " << calibrator.getNumberOfViews() << endl;
    cout << "    calibrator info: " << calibrator.getInfo() << endl;
    cout << "    save camera parameters to: " << file_path << endl;
    cout << endl;
    
    calibrator.getCalibrationResults(camp);
    camp.saveToFile(file_path);
}

void calibrateCameraWithVideo(const string& file_path, const string& calibration_video_path, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map){
    // 获得图像和marker位置
    aruco::MarkerDetector marker_detector(aruco::Dictionary::ARUCO_MIP_36h12);  //检测marker
    vector<vector<aruco::Marker>> markers_vec;
    cv::Size img_size(image_width, image_height);
    
    aruco::Calibrator calibrator;               //标定
    aruco::CameraParameters camp;               //相机参数
    calibrator.setParams(img_size, calibration_marker_size, calibration_marker_map);
    
    cv::VideoCapture vc(calibration_video_path);
    assert(vc.isOpened());
    while (vc.grab()) {
        cv::Mat img;
        vc.retrieve(img);
        vector<aruco::Marker> markers =  marker_detector.detect(img);   //检测marker
        calibrator.addView(markers);
    }
    
    cout << "calibrate camera" << endl;
    cout << "    num of views: " << calibrator.getNumberOfViews() << endl;
    cout << "    calibrator info: " << calibrator.getInfo() << endl;
    cout << "    save camera parameters to: " << file_path << endl;
    cout << endl;
    
    calibrator.getCalibrationResults(camp);
    camp.saveToFile(file_path, true);
}

// 正十二面体标定
void calibrateDodecaWithImages(const string& file_path_base_name, const string& calibration_photo_path, int image_num, float calibration_marker_size, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary){
    auto amm = aruco_mm::MarkerMapper::create();
    amm->setParams(camera_parameters, calibration_marker_size);
    amm->getMarkerDetector().setDictionary(dictionary);
    amm->getMarkerDetector().setDetectionMode(aruco::DM_NORMAL);
    for (int i=0; i<image_num; i++) {
        string path = calibration_photo_path+"/"+to_string(i)+".jpg";
        cv::Mat img = cv::imread(path);
        assert(!img.empty());
        amm->process(img, i);
    }
    amm->optimize();
    
    cout << "calibrate dodecahedron" << endl;
    cout << "    save aruco_mm::MarkerMapper to: " << file_path_base_name << endl;
    cout << "    save pcd to: " << file_path_base_name+".pcd" << endl;
    cout << "    save frame poses to: " << file_path_base_name+".log" << endl;
    cout << "    save camera parameters to: " << file_path_base_name+"-cam.yml" << endl;
    cout << "    save markermap to: " << file_path_base_name+".yml" << endl;
    cout << endl;
    amm->saveToFile(file_path_base_name);
    amm->saveToPcd(file_path_base_name+".pcd", true);
    amm->saveFrameSetPosesToFile(file_path_base_name+".log");
    amm->getCameraParams().saveToFile(file_path_base_name+"-cam.yml");
    amm->getMarkerMap().saveToFile(file_path_base_name+".yml");
}
