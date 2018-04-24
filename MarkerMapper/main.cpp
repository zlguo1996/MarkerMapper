//
//  main.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/3/19.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "stdafx.h"

#include "sglviewer.h"

#include "Calibration.hpp"
#include "Camera.hpp"
#include "Pen.hpp"
#include "PenDetector.hpp"

//#define CALIB_CAM
//#define CALIB_DODECA
#define REALTIME_TRACK

// 相机标定变量
float calibration_marker_size = 0.018;    //标定用marker的边长

// 正十二面体变量
aruco::Dictionary::DICT_TYPES dictionary = aruco::Dictionary::ARUCO_MIP_36h12;  //正十二面体表面marker所属的字典
float dodeca_marker_size = 0.010;       //正十二面体表面marker边长

void visualizeMap(const string& map_path){
    cout << "visualize map: press ESC to escape" << endl;
    OpenCvMapperViewer viewer;
    aruco::MarkerMap mmap;
    mmap.readFromFile(map_path);
    viewer.setParams(mmap, 1.5, 1080, 720, "map_viewer");
    int key=0;
    while (key!=27) {
        key = viewer.show();
    }
}

int main(int argc, const char * argv[]) {
    string camera_parameters_file_path = "Calibration/output/camera/logitech_brio_camera_calibration_1080p.yml";
#ifdef CALIB_CAM
    // 标定相机，并保存相机参数到文件（api学习：aruco/utils_calibration/aruco_calibration.cpp）
    //string calibration_video_path = "Calibration/input/macbook_camera_calibration.mov";
    string calibration_photo_path = "Calibration/input/logitech_brio_camera_calibration/1080p";
    calibrateCameraWithImages(camera_parameters_file_path, calibration_photo_path, 42, 1920, 1080, calibration_marker_size);
    //calibrateCameraWithVideo(camera_parameters_file_path, calibration_video_path, 1080, 720, calibration_marker_size);
#endif
    
    // 读取相机参数文件
    aruco::CameraParameters camera_parameters;
    camera_parameters.readFromXMLFile(camera_parameters_file_path);
    
    string marker_map_path_base_name = "Calibration/output/dodeca/dodecahedron_marker_map";
#ifdef CALIB_DODECA
    // 创建map（api学习：markermapper/utils/mapper_from_images.cpp）
    string dodecahedron_photo_path = "Calibration/input/dodecahedron_calibration";
    calibrateDodecaWithImages(marker_map_path_base_name, dodecahedron_photo_path, 45, calibration_marker_size, camera_parameters, dictionary);
#endif
    
    string board_marker_map_path = "Calibration/output/board_marker_map.yml";
    
    //可视化map
    //visualizeMap(marker_map_path_base_name+".yml");
#ifdef REALTIME_TRACK
    //追踪
    aruco::MarkerMap mmap;
    mmap.readFromFile(marker_map_path_base_name+".yml");
    assert(mmap.isExpressedInMeters());
    aruco::CameraParameters cp;
    cp.readFromXMLFile(marker_map_path_base_name+"-cam.yml");
    assert(cp.isValid());
    
    aruco::MarkerDetector md;
    md.setDictionary(dictionary);
    md.getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    
    string case_path = "Tracking/case1.mov";
    cv::VideoCapture video_capture(0);
    //video_capture.set(CV_CAP_PROP_FPS, 30);
    cout << video_capture.get(CV_CAP_PROP_FPS) << endl;
    
    Camera camera(camera_parameters_file_path, video_capture, cv::Mat::eye(4, 4, CV_32F), md);
    camera.setVideoCaptureParameters();
    Pen pen(marker_map_path_base_name+".yml");
    PenDetector pd(&camera,&pen);
    
    while (camera.grab()) {
        bool success = pd.detectOneFrame();
        Mat img;
        pd.camera->current_frame.copyTo(img);
        if (success) {
            pd.camera->drawDetectedMarkers(img);
            pd.camera->draw3DAxis(img, dodeca_marker_size*2);
        }
        
//        aruco::MarkerMapPoseTracker mmappt;
//        mmappt.setParams(cp, mmap);
//        if(mmappt.estimatePose(markers)){
//            aruco::CvDrawingUtils::draw3dAxis(img, cp, mmappt.getRvec(), mmappt.getTvec(), dodeca_marker_size*2);
//        }
//        for(auto i:markers) i.draw(img);
        
        imshow("in", img);
        char c = waitKey(20);
        if(c==27) break;
    }
#endif
    
    return 0;
}
