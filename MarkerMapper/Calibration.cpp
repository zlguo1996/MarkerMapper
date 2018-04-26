//
//  Calibration.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/21.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "Calibration.hpp"

// 读取文件夹中以自然数命名的图片
bool readImages(const string& photo_path, vector<Mat>& images){
    if (!images.empty()) {
        cout << "READIMAGES::ERROR::Images should be empty." << endl;
        return false;
    }
    if (access(photo_path.c_str(), F_OK)==-1) {
        cout << "READIMAGES::ERROR::No such folder." << endl;
        return false;
    }
    
    int idx = 0;
    Mat img = imread(photo_path+"/"+to_string(idx)+".jpg");
    while (!img.empty()) {
        images.push_back(img);
        idx++;
        img = imread(photo_path+"/"+to_string(idx)+".jpg");
    }
    
    if(images.size()==0){
        cout << "READIMAGES::ERROR::No images in folder." << endl;
        return false;
    }
    return true;
}

// 相机标定
void calibrateCameraWithImages(const string& file_path, const string& calibration_photo_path, int image_width, int image_height, float calibration_marker_size, const string& calibration_marker_map){
    // 获得图像和marker位置
    aruco::MarkerDetector marker_detector(aruco::Dictionary::ARUCO_MIP_36h12);  //检测marker
    cv::Size img_size(image_width, image_height);
    
    aruco::Calibrator calibrator;               //标定
    aruco::CameraParameters camp;               //相机参数
    calibrator.setParams(img_size, calibration_marker_size, calibration_marker_map);
    
    vector<Mat> images;
    readImages(calibration_photo_path, images);
    for (int i=0; i<images.size(); i++) {
        vector<aruco::Marker> markers =  marker_detector.detect(images[i]);   //检测marker
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
void calibrateDodecaWithImages(const string& file_path_base_name, const string& calibration_photo_path, float calibration_marker_size, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary){
    auto amm = aruco_mm::MarkerMapper::create();
    amm->setParams(camera_parameters, calibration_marker_size);
    amm->getMarkerDetector().setDictionary(dictionary);
    amm->getMarkerDetector().setDetectionMode(aruco::DM_NORMAL);
    
    vector<Mat> images;
    readImages(calibration_photo_path, images);
    for (int i=0; i<images.size(); i++) {
        amm->process(images[i], i);
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

// 笔尖标定
void calibratePentip(const string& file_path, const string& calibration_photo_path, const aruco::CameraParameters& camera_parameters, const aruco::Dictionary::DICT_TYPES& dictionary, const aruco::MarkerMap& marker_map){
    vector<Mat> images;
    readImages(calibration_photo_path, images);
    
    aruco::MarkerDetector md;
    md.setDictionary(dictionary);
    md.getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    
    aruco::MarkerMapPoseTracker mmpt;
    mmpt.setParams(camera_parameters, marker_map);
    assert(mmpt.isValid());
    
    double caliDofs[6*images.size()];
    for(int i=0; i<images.size(); i++){
        // 检测marker
        aruco_mm::arucoMarkerSet marker_set = md.detect(images[i]);
        assert(marker_set.size()>=2);

        // 得到相机姿态
        mmpt.estimatePose(marker_set);
        Mat rvec = mmpt.getRvec();
        Mat tvec = mmpt.getTvec();
        DOF_6 dof;
        assert(rvec.type()==CV_32F && tvec.type()==CV_32F);
        assert(rvec.rows==1 && tvec.rows==1);
        dof[0] = rvec.at<float>(0, 0);
        dof[1] = rvec.at<float>(0, 1);
        dof[2] = rvec.at<float>(0, 2);
        dof[3] = tvec.at<float>(0, 0);
        dof[4] = tvec.at<float>(0, 1);
        dof[5] = tvec.at<float>(0, 2);
        memcpy(&caliDofs[6*i], dof, 6*sizeof(double));
    }
    
    double ptPosition[3] = {0.05, 0.05, 0.05};
    ceres::Problem problem;
    for (int i=0; i<images.size()-1; i++) {
        for (int j=i; j<images.size(); j++){
            ceres::CostFunction* cost_function = ReprojectionErrorPtClb::Create(&caliDofs[i*6], &caliDofs[j*6]);
            problem.AddResidualBlock(cost_function, NULL, ptPosition);
        }
    }
    
    // options
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
}
